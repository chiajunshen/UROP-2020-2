classdef MILP_unsig_int_1_Controller < matlab.DiscreteEventSystem & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.CustomIcon
    
    properties (Nontunable)
        capacity = 100;         % Control zone capacity
        simulation_step = 0.01; % Simulation step
        MILP_step = 4;          % run MILP every 4 seconds        
        L = 500;                % distance (subscription point and centre of intersection)
        d_access = 40;          % distance (access point and merging zone)
        S = 30;                 % size of merging zone        
        
        % MILP -- values taken from YR's paper
        a_acc_max   = 3;      % maximum acceleration
        v_max       = 19;     % maximum speed limit
        v_ave       = 15.64;  % average velocity
        a_dec_max   = -4;     % maximum emergency deceleration        
    end
    
    properties (DiscreteState)
        firstVehicle_S1;
        firstVehicle_S3;
        
        first_vehicle_ID;
        first_vehicle_Lane;
        first_vehicle_Position;
        first_vehicle_Speed;
        first_vehicle_Intersection;
    end
    
    properties
        % optimal speed/position profiles
        profiles = repmat(struct('speed', @(t) 1, 'position', @(t) 1), 1000, 1);
        % iteration start mark
        storageVisited = zeros(5, 1);
        % check if info is received by CAV i
        %info_received = zeros(1000, 1);
        
        % MILP        
        cavNum = 0;        
        arrayMinAccessTime = zeros(1000, 1);
        arrayLane = zeros(1000, 1);
        arraySolvedMILP = zeros(1000, 1);
        
        run_MILP = 0;
        ready_to_update = 0;
        index = 0;
        delay = 0;
    end
    
    methods (Access=protected)
        
        function num = getNumInputsImpl(~)
            % Define number of inputs for system with optional inputs
            num = 2;
        end
        
        function num = getNumOutputsImpl(~)
            % Define number of outputs for system with optional outputs
            num = 2;
        end
        
        function entityTypes = getEntityTypesImpl(obj)
            % Define entity types being used in this model
            entityTypes(1) = obj.entityType('CAV', 'CAV', 1, false);
            entityTypes(2) = obj.entityType('INFO', 'INFO', 1, false);
        end
        
        function [input, output] = getEntityPortsImpl(~)
            % Define data types for entity ports
            input = {'CAV','INFO'};
            output = {'CAV','INFO'};
        end
        
        function [storageSpec, I, O] = getEntityStorageImpl(obj)
            % CAV - cruise 
            storageSpec(1) = obj.queueFIFO('CAV', obj.capacity);
            % INFO
            storageSpec(2) = obj.queueFIFO('INFO', obj.capacity);
            % CAV - follow trajectory 
            storageSpec(3) = obj.queueFIFO('CAV', obj.capacity);
            
            I = [1 2];
            O = [3 2];
        end
        
        function sz = getOutputSizeImpl(~)
            % Return size for each output port
            sz(1) = 1;
            sz(2) = 1;
        end
        
        function dt = getOutputDataTypeImpl(~)
            % Return data type for each output port
            dt(1) = 'CAV';
            dt(2) = 'INFO';
        end
        
        function cp = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            cp(1) = false;
            cp(2) = false;
        end
       
        function [name1, name2] = getInputNamesImpl(~)
            % Return input port names for System block
            name1 = 'IN';
            name2 = 'INFO';
        end
        
        function [name1, name2] = getOutputNamesImpl(~)
            % Return input port names for System block
            name1 = 'OUT';
            name2 = 'INFO';
        end
        
        function icon = getIconImpl(~)
            icon = sprintf('MILP\nCONTROL ZONE');
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, ~)
            sz = 1;
            dt = 'double';
            cp = false;
        end
        
        function setupImpl(obj)
            obj.firstVehicle_S1 = 0;            
            obj.firstVehicle_S3 = 0;
        end
        
        function [entity, events] = CAVGenerateImpl(obj, storage, entity, tag)
            switch tag                    
                case 'copy_first_vehicle_S1toS3'
                    % generate a entity and copy the properties of CAV #1
                    entity.data.ID = obj.first_vehicle_ID;
                    entity.data.Lane = obj.first_vehicle_Lane;
                    entity.data.Position = obj.first_vehicle_Position;
                    entity.data.Speed = obj.first_vehicle_Speed;                    
                    entity.data.Intersection = obj.first_vehicle_Intersection;
                    events = obj.eventForward('storage', 3, 0);
                    
                case 'copy_first_vehicle_S3toMZ'
                    % generate a entity and copy the properties of CAV #1
                    entity.data.ID = obj.first_vehicle_ID;
                    entity.data.Lane = obj.first_vehicle_Lane;
                    entity.data.Position = obj.first_vehicle_Position;
                    entity.data.Speed = obj.first_vehicle_Speed;                    
                    entity.data.Intersection = obj.first_vehicle_Intersection;
                    events = obj.eventForward('output', 1, 0);                    
            end
        end
        
        function [entity, events] = CAVEntryImpl(obj, storage, entity, ~)
            if storage == 1
                if obj.storageVisited(1) == 0
                    obj.storageVisited(1) = 1;
                    events = [obj.eventTimer('run_MILP',obj.simulation_step), ...
                        obj.eventTimer('cruising', obj.simulation_step)];
                else
                    events = [];
                end
              
            elseif storage == 3            
                if obj.storageVisited(3) == 0
                    obj.storageVisited(3) = 1;
                    events = obj.eventTimer('follow_trajectory',obj.simulation_step);
                else
                    events = [];
                end
            end
        end
        
        function [entity, events] = CAVTimerImpl(obj, storage, entity, tag)
            events = [];
            switch tag
                case 'run_MILP'
                    if obj.run_MILP == 0 
                        obj.run_MILP = 1;
                        events = obj.eventTimer('run_MILP', obj.MILP_step);
                    else
                        events = [];
                    end                                        
                    
                case 'cruising'
                    events = [obj.eventIterate(1, 'cruise', 1), ...
                        obj.eventTimer('cruising', obj.simulation_step)];
                    
                case 'follow_trajectory'
                    events = [obj.eventIterate(3, 'CZ', 1), ...
                        obj.eventTimer('follow_trajectory', obj.simulation_step)];                                       
            end
        end
        
        function [entity, events, next] = CAVIterateImpl(obj, storage, entity, tag, status)
            events = [];
            switch tag
                case 'cruise'
                    if entity.data.ID ~= 1001
                        [entity.data.Position, entity.data.Speed] = ...
                            MILP_getCruiseStatus(entity.data.Position, ...
                            entity.data.Speed, obj.simulation_step);
                        MILP_unsig_int_1_plotCAV(entity.data.Position, entity.data.Lane, ...
                            entity.data.ID, entity.data.Intersection);
                        
                        if obj.run_MILP == 1
                            if  obj.firstVehicle_S1 == 0
                                obj.first_vehicle_ID = entity.data.ID;
                                obj.first_vehicle_Intersection = entity.data.Intersection;
                                obj.first_vehicle_Lane = entity.data.Lane;
                                obj.first_vehicle_Position = entity.data.Position;
                                obj.first_vehicle_Speed = entity.data.Speed;
                                entity.data.ID = 1001;
                                events = obj.eventGenerate(3, 'copy_first_vehicle_S1toS3', 0, 1);
                                obj.firstVehicle_S1 = 1;
                            else
                                events = obj.eventForward('storage', 3, 0);
                            end
                        end                        
                    end
                    next = true;
                    
                case 'CZ'
                    if entity.data.ID ~= 1001
                        
                        if (obj.run_MILP == 1) && (obj.ready_to_update == 0)
                            % perform MILP
                            if entity.data.Position > 0
                                obj.cavNum = obj.cavNum + 1;
                                obj.arrayLane(obj.cavNum) = entity.data.Lane;
                                obj.arrayMinAccessTime(obj.cavNum) = ...
                                    compute_t_access_min(entity.data.Position, entity.data.Speed, ...
                                    obj.v_max, obj.a_acc_max);
                            end
                            
                            if status.position == status.size
                                [obj.arraySolvedMILP, obj.delay] = solvemymilp(obj.cavNum, obj.arrayLane, ...
                                    obj.arrayMinAccessTime);
                                obj.cavNum = 0;
                                obj.arrayLane(:) = 0;
                                obj.arrayMinAccessTime(:) = 0;
                                obj.run_MILP = 0;
                                obj.ready_to_update = 1;
                                events = obj.eventIterate(3, 'CZ', 1);
                            end
                        elseif (obj.run_MILP == 0) && (obj.ready_to_update == 1)
                            % consider the computation time of MILP
%                             [entity.data.Position, entity.data.Speed] = ...
%                                 MILP_getCruiseStatus(entity.data.Position, entity.data.Speed, obj.delay);
%                             MILP_unsig_int_1_plotCAV(entity.data.Position, entity.data.Lane, ...
%                                 entity.data.ID, entity.data.Intersection);
                            
                            obj.index = obj.index + 1;
                            if entity.data.Position > 0
                                entity.data.FinalTime = obj.arraySolvedMILP(obj.index + 1);
                                [obj.profiles(entity.data.ID).position, obj.profiles(entity.data.ID).speed] = ...
                                    trajectory_planning(entity.data.Position, entity.data.Speed, ...
                                    entity.data.FinalTime, obj.delay);
%                                 [obj.profiles(entity.data.ID).speed, obj.profiles(entity.data.ID).position] = ...
%                                     trajectory_planning_OC_MOD(entity.data.Speed, 10, ...
%                                     entity.data.FinalTime, entity.data.Position, obj.delay);
                            else
                                obj.index = obj.index - 1;
                            end
                            
                            if status.position == status.size
                                obj.ready_to_update = 0;
                                obj.index = 0;
                                events = obj.eventIterate(3, 'CZ', 1);
                            end
                        else
                            % normal plotting
                            [entity.data.Position, entity.data.Speed] = ...
                                MILP_getStatus(obj.profiles(entity.data.ID).position, ...
                                obj.profiles(entity.data.ID).speed);
                            MILP_unsig_int_1_plotCAV(entity.data.Position, entity.data.Lane, ...
                                entity.data.ID, entity.data.Intersection);
                            
                            % entering the access area
                            % Merging Zone in SIMULINK DES = access
                            % area + merging zone in YR's paper
                            if entity.data.Position < 0
                                if  obj.firstVehicle_S3 == 0
                                    obj.first_vehicle_ID = entity.data.ID;
                                    obj.first_vehicle_Intersection = entity.data.Intersection;
                                    obj.first_vehicle_Lane = entity.data.Lane;
                                    obj.first_vehicle_Position = entity.data.Position;
                                    obj.first_vehicle_Speed = entity.data.Speed;
                                    entity.data.ID = 1001;
                                    events = obj.eventGenerate(3, 'copy_first_vehicle_S3toMZ', 0, 1);
                                    obj.firstVehicle_S3 = 1;
                                else
                                    events = obj.eventForward('output', 1, 0);
                                end
                            end
                        end                        
                    end
                    next = true;
            end            
        end
        
    end
    
end

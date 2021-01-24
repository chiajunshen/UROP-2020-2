classdef MILP_unsig_int_1_RoadSegment_4 < matlab.DiscreteEventSystem & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.CustomIcon
    
    properties (Nontunable)
        capacity = 100;         % Control zone capacity
        simulation_step = 0.01; % Simulation step       
        L = 500;                % distance (subscription point and centre of intersection)
        d_access = 40;          % distance (access point and merging zone)
        S = 30;                 % size of merging zone
    end
    
    properties (DiscreteState)
        FirstVehicle;
        start_ID;
        
        first_vehicle_ID;
        first_vehicle_Lane;
        first_vehicle_Position;
        first_vehicle_Speed;
        first_vehicle_Intersection;
    end    
    
    methods (Access=protected)
        
        function num = getNumInputsImpl(~)
            % Define number of inputs for system with optional inputs
            num = 1;
        end
        
        function num = getNumOutputsImpl(~)
            % Define number of outputs for system with optional outputs
            num = 1;
        end
        
        function entityTypes = getEntityTypesImpl(obj)
            % Define entity types being used in this model
            entityTypes = obj.entityType('CAV', 'CAV', 1, false);
        end
        
        function [input, output] = getEntityPortsImpl(~)
            % Define data types for entity ports
            input = {'CAV'};
            output = {'CAV'};
        end
        
        function [storageSpec, I, O] = getEntityStorageImpl(obj)
            % CAV -> optimal control implemented
            storageSpec = obj.queueFIFO('CAV', obj.capacity);
            I = 1;
            O = 1;
        end
        
        function sz = getOutputSizeImpl(~)
            % Return size for each output port
            sz = 1;
        end
        function dt = getOutputDataTypeImpl(~)
            % Return data type for each output port
            dt = 'CAV';
        end
        
        function cp = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            cp = false;
        end
        
        function name = getInputNamesImpl(~)
            % Return input port names for System block
            name = 'IN';
        end
        
        function name = getOutputNamesImpl(~)
            % Return input port names for System block
            name = 'OUT';
        end
        function icon = getIconImpl(~)
            icon = sprintf('LEAVE THE INTERSECTION');
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, ~)
            sz = 1;
            dt = 'double';
            cp = false;
        end
        
        function resetImpl(obj)
            obj.FirstVehicle = 0;
            obj.start_ID = 0;
            
            obj.first_vehicle_ID = 0;
            obj.first_vehicle_Lane = 0;
            obj.first_vehicle_Position = 0;
            obj.first_vehicle_Speed = 0;
            obj.first_vehicle_Intersection = 0;
        end
        
        function [entity, events] = CAVGenerateImpl(obj, storage, entity, tag)
            switch tag
                case 'copy_first_vehicle'
                    % generate a entity and copy the properties of CAV #1
                    entity.data.ID = obj.first_vehicle_ID;
                    entity.data.Lane = obj.first_vehicle_Lane;
                    entity.data.Position = obj.first_vehicle_Position;
                    entity.data.Speed = obj.first_vehicle_Speed;
                    events = obj.eventForward('output', 1, 0);
            end
        end
        
        function [entity, events] = CAVEntryImpl(obj, storage, entity, ~)
            % CAV waiting for ID assigned by the coordinator
            if storage == 1 % CAV enters the CZ
                if obj.start_ID == 0
                    obj.start_ID = entity.data.ID;
                    events = obj.eventTimer('cruise',obj.simulation_step);
                else
                    events = [];
                end
            end
        end
        
        function [entity, events] = CAVTimerImpl(obj, storage, entity, tag)
            events = [];
            switch tag
                case 'cruise' % control zone - control
                    events = [ obj.eventIterate(1, 'road_segment', 1), ...
                        obj.eventTimer('cruise',obj.simulation_step) ];
            end
        end
        
        function [entity, events, next] = CAVIterateImpl(obj, storage, entity, tag, status)
            events = [];
            switch tag
                case 'road_segment'
                    % compute the dynamics based on the latest control
                    % cruise
                    if entity.data.ID ~= 1001
                        [entity.data.Position, entity.data.Speed] = ...
                            MILP_getCruiseStatus(entity.data.Position, entity.data.Speed, ...
                            obj.simulation_step);
                        
                        MILP_unsig_int_1_plotCAV(entity.data.Position, entity.data.Lane, ...
                            entity.data.ID, entity.data.Intersection);
                        
                        % leaving road segment (leaving intersection)
                        if abs(entity.data.Position) > obj.L + obj.S/2 + obj.d_access + 1
                            if  obj.FirstVehicle == 0
                                obj.first_vehicle_ID = entity.data.ID;
                                obj.first_vehicle_Lane = entity.data.Lane;
                                obj.first_vehicle_Position = entity.data.Position;
                                obj.first_vehicle_Speed = entity.data.Speed;
                                entity.data.ID = 1001;
                                events = obj.eventGenerate(1, 'copy_first_vehicle', 0, 1);
                                obj.FirstVehicle = 1;
                            else
                                events = obj.eventForward('output', 1, 0);
                            end
                        end
                    end
                    next = true;
            end
        end
        
    end
    
end
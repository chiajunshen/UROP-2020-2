classdef YR
    
    properties
        cavNum;
        arrayMinAccessTime;
        arrayCarLane;
        arraySolvedMILP;
    end
    
    
    
    methods
        function self = YR
            self.cavNum = 0;
        end
        
        function Increment(self, t_access_min, lane)
            if nargin > 2
                self.cavNum = self.cavNum + 1;
                self.arrayMinAccessTime(self.cavNum) = t_access_min;
                self.arrayCarLane(self.cavNum) = lane;
                self.arraySolvedMILP = solvemymilp(self.cavNum, self.arrayCarLane, self.arrayMinAccessTime);
            end
        end
        
        function Decrement(self)
            self.cavNum = self.cavNum - 1;
            self.arrayMinAccessTime(1) = [];
            self.arrayCarLane(1) = [];
        end
        
%         function toSolveMILP(self)
%             self.arraySolvedMILP = solvemymilp(self.cavNum, self.arrayCarLane, self.arrayMinAccessTime);
%         end
        
    end
end
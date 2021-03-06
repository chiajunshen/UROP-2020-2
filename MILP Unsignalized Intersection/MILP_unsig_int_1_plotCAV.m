function MILP_unsig_int_1_plotCAV(pos, lane, id, intersection)

global p1
global p1_color
global p2
global p2_color

% parameter from YR's paper
Len = 500;      % distance between subscription point and centre of intersection    
S = 30;         % size of merging zone
d = 40;         % distance between access point and merging zone
centre = 0;     % centre = (x,y) = (0,0)

% redefine pos (position) for plotting
pos = Len - d - S/2 - pos;

% l_center = -215;
% r_center = 215;
%if intersection == 1
    switch lane
        case 1
            if p1_color(id) == 0
                p1(id).MarkerEdgeColor = [0 0.4470 0.7410];
                p1(id).MarkerFaceColor = [0 0.4470 0.7410];
                p1_color(id) = 1;
                p1(id).YData = -S/4;
            end
            p1(id).XData = pos + centre - Len;
  
        case 2
            if p1_color(id) == 0
                p1(id).MarkerEdgeColor = [0.8500 0.3250 0.0980];
                p1(id).MarkerFaceColor = [0.8500 0.3250 0.0980];
                p1_color(id) = 1;
                p1(id).XData = centre + S/4;
            end
            p1(id).YData = pos - Len;
            
        case 3
            if p1_color(id) == 0
                p1(id).MarkerEdgeColor = [0.9290 0.6940 0.1250];
                p1(id).MarkerFaceColor= [0.9290 0.6940 0.1250];
                p1_color(id) = 1;
                p1(id).YData = S/4;
            end
            p1(id).XData = - pos + Len + centre;   
            
        case 4
            if p1_color(id) == 0
                p1(id).MarkerEdgeColor = [0.4940 0.1840 0.5560];
                p1(id).MarkerFaceColor = [0.4940 0.1840 0.5560];
                p1_color(id) = 1;
                p1(id).XData = centre - S/4;
            end
            p1(id).YData = - pos + Len;
            
    end
% else
%     switch lane
%         case 1
%             if p2_color(id) == 0
%                 p2(id).MarkerEdgeColor = [0 0.4470 0.7410];
%                 p2(id).MarkerFaceColor = [0 0.4470 0.7410];
%                 p2_color(id) = 1;
%                 p2(id).YData = -7.5;
%             end
%             p2(id).XData = pos + r_center - 415;
% 
%         case 2
%             if p2_color(id) == 0
%                 p2(id).MarkerEdgeColor = [0.4660 0.6740 0.1880];
%                 p2(id).MarkerFaceColor = [0.4660 0.6740 0.1880];
%                 p2_color(id) = 1;
%                 p2(id).XData = r_center + 7.5;
%             end
%             p2(id).YData = pos - 415;
%             
%         case 3
%             if p2_color(id) == 0
%                 p2(id).MarkerEdgeColor = [0.9290 0.6940 0.1250];
%                 p2(id).MarkerFaceColor= [0.9290 0.6940 0.1250];
%                 p2_color(id) = 1;
%                 p2(id).YData = 7.5;
%             end
%             p2(id).XData = - pos + 415 + r_center;
%             
%         case 4
%             if p2_color(id) == 0
%                 p2(id).MarkerEdgeColor = [0.3010 0.7450 0.9330];
%                 p2(id).MarkerFaceColor = [0.3010 0.7450 0.9330];
%                 p2_color(id) = 1;
%                 p2(id).XData = r_center -7.5;
%             end  
%             p2(id).YData = - pos + 415;
%             
%     end
% end
end
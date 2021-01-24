% this is for Centralized MILP at unsig_int_1

function MILP_unsig_int_1_plotBGIMAGE()
close all

global p1
global p1_color
global p2
global p2_color

% parameter from YR's paper
Len = 500;      % distance between subscription point and centre of intersection    
S = 30;         % size of merging zone
d = 40;         % distance between access point and merging zone
centre = 0;     % centre = (x,y) = (0,0)

roadLineWidth = 1;
centreLineWidth = 0.5; 

% plot the road line
plot([centre + S/2, centre + S/2],[S/2, Len],'k', 'linewidth', roadLineWidth)
hold on
plot([centre + S/2, centre + S/2],[-S/2, -Len],'k', 'linewidth', roadLineWidth)
hold on
plot([centre - S/2, centre - S/2],[S/2, Len],'k', 'linewidth', roadLineWidth)
hold on
plot([centre - S/2, centre - S/2],[-S/2, -Len],'k', 'linewidth', roadLineWidth)
hold on
plot([centre + S/2, centre + Len], [S/2, S/2],'k', 'linewidth', roadLineWidth)
hold on
plot([centre - S/2, centre - Len],[S/2, S/2],'k', 'linewidth', roadLineWidth)
hold on
plot([centre + S/2, centre + Len], [-S/2, -S/2],'k', 'linewidth', roadLineWidth)
hold on
plot([centre - S/2, centre - Len],[-S/2, -S/2],'k', 'linewidth', roadLineWidth)
hold on

% plot the centerline
plot([-Len, centre-S/2],[centre,centre],'--k', 'linewidth', centreLineWidth)
hold on 
plot([centre+S/2, centre+Len],[centre,centre],'--k', 'linewidth', centreLineWidth)
hold on 
plot([centre, centre],[S/2, Len],'--k', 'linewidth', centreLineWidth)
hold on 
plot([centre, centre],[-Len, -S/2],'--k', 'linewidth', centreLineWidth)
hold on

% plot the access area's lines
dis_from_centre = centre + S/2 + d;
plot([dis_from_centre, dis_from_centre],[dis_from_centre,-dis_from_centre],'--r', 'linewidth', centreLineWidth)
hold on
plot([dis_from_centre, -dis_from_centre],[-dis_from_centre,-dis_from_centre],'--r', 'linewidth', centreLineWidth)
hold on 
plot([-dis_from_centre, -dis_from_centre],[-dis_from_centre,dis_from_centre],'--r', 'linewidth', centreLineWidth)
hold on 
plot([-dis_from_centre, dis_from_centre],[dis_from_centre,dis_from_centre],'--r', 'linewidth', centreLineWidth)
hold on 

% map size
axis([-Len, Len, -Len, Len])
axis('square')
%
% plot the vehicles
N = 1001;
y = zeros(N, N) + 1000; %1000: number of CAVs
p1 = plot(y,'o','MarkerSize',5);
p2 = plot(y,'o', 'MarkerSize',5);
p1_color = zeros(N, 1); % check if a CAV has determined its color
p2_color = zeros(N, 1); % check if a CAV has determined its color
hold on

% vehicle legend
LH(1) = plot(nan, nan, 'o', 'MarkerSize',5,'MarkerEdgeColor',[0 0.4470 0.7410],...
    'MarkerFaceColor',[0 0.4470 0.7410]);
LH(2) = plot(nan, nan, 'o', 'MarkerSize',5,'MarkerEdgeColor',[0.8500 0.3250 0.0980],...
    'MarkerFaceColor',[0.8500 0.3250 0.0980]);
LH(3) = plot(nan, nan, 'o', 'MarkerSize',5,'MarkerEdgeColor',[0.9290 0.6940 0.1250],...
    'MarkerFaceColor',[0.9290 0.6940 0.1250]);
LH(4) = plot(nan, nan, 'o', 'MarkerSize',5,'MarkerEdgeColor',[0.4940 0.1840 0.5560],...
    'MarkerFaceColor',[0.4940 0.1840 0.5560]);

L(1) = "vehicle from West";
L(2) = "vehicle from South";
L(3) = "vehicle from East";
L(4) = "vehicle from North";

legend(LH, L);
end
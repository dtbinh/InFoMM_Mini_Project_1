function [] = bearing_measure_simple(T)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is an implementation of the initial bearing-based measurement
%     method. We choose certain parameters, to create a visual of how the
%     parameters theta(t) and z(t) change over time.
% INPUT: 
%     T: {scalar} Total amount of time to run the visualisation.
% OUTPUT:
%     : {}

%% Examples
% [] = bearing_measure_simple()

%%
keepvars = {'T'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

dt = 0.001;
omega1 = 8*rand;
omega2 = 5*rand;
phi1 = rand;
phi2 = rand;
rad1 = ceil(5*rand);
rad2 = floor(8*rand);

circle1 = @(t) [sin(omega1*t + phi1), cos(omega1*t + phi1)];
circle2 = @(t) [sin(omega2*t + phi2), cos(omega2*t + phi2)] + [5,2];

timesteps = T/dt + 1;
all_time = linspace(0,T,timesteps);

%%

bearing_plot = figure();
g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
g3 = animatedline('Color','r','MaximumNumPoints',200);
g4 = animatedline('Color','b','MaximumNumPoints',200);
legend('Drone 1','Drone 2')
axis([-1,7,-2,4]);
shg;

for i = 1:length(all_time)
    t = all_time(i);
    P = circle1(t);
    Q = circle2(t);
    addpoints(g1,P(1),P(2));
    addpoints(g2,Q(1),Q(2));
    addpoints(g3,P(1),P(2));
    addpoints(g4,Q(1),Q(2));
    hold on
    ang_line = plot([P(1),Q(1)],[P(2),Q(2)],'k--');
    drawnow;
    delete(ang_line);
end

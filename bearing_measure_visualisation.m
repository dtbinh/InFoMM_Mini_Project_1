function [] = bearing_measure_visualisation(T,dt)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is an implementation of the initial bearing-based measurement
%     method. We choose certain parameters, to create a visual of how the
%     parameters theta(t) and z(t) change over time.
% INPUT: 
%     T: {float} Total amount of time to run the visualisation. Will
%     probably be taken as an integer.
%    dt: {float} Timestep to be taken.
% OUTPUT:
%      : {}

%% Example
% [] = bearing_measure_visualisation(100,0.002)

%%
keepvars = {'T','dt'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

% Initialise with two uniformly random angular speeds, phases, and radii.
omega1 = 8*rand;
omega2 = 5*rand;
phi1 = rand;
phi2 = rand;
rad1 = ceil(5*rand);
rad2 = floor(8*rand);

% Functional forms of the circular paths.
circle1_F = @(t) [sin(omega1*t + phi1), cos(omega1*t + phi1)];
circle2_F = @(t) [sin(omega2*t + phi2), cos(omega2*t + phi2)] + [5,2];

% 
timesteps = T/dt + 1;
all_time_V = linspace(0,T,timesteps);

% Set up the animations to be drawn.
angular_plot = subplot(1,2,2);
a1 = animatedline('Color','k');
legend('Angle');
axis([0,T,-pi/2,pi/2]);
shg;
%
bearing_plot = subplot(1,2,1);
g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
g3 = animatedline('Color','r','MaximumNumPoints',200);
g4 = animatedline('Color','b','MaximumNumPoints',200);
legend('Drone 1','Drone 2')
axis([-1,7,-2,4]);


% Loop over all time to plot the trajectories, as well as the connecting
% line between the two drones.
for i = 1:length(all_time_V)
    %
    t = all_time_V(i);
    P = circle1_F(t);
    Q = circle2_F(t);
    
    % Angular plot.
    addpoints(a1,t,atan((Q(2) - P(1))/(Q(1) - P(1))));
    
    % Circular plots.
    addpoints(g1,P(1),P(2));
    addpoints(g2,Q(1),Q(2));
    addpoints(g3,P(1),P(2));
    addpoints(g4,Q(1),Q(2));
    hold on
    %
    % Draw AND remove the line one at a time.
    ang_line = plot([P(1),Q(1)],[P(2),Q(2)],'k--');
    hor_line = plot([P(1),P(1)+1],[P(2),P(2)],'k--');
    drawnow;
    delete(ang_line);
    delete(hor_line);
    
end

end

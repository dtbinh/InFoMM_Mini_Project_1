function [] = planar_tracking_with_four_drones_TEST()

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     four individual drones and a single object to be tracked. In this
%     formulation, we will impose all parameters, as well as the movement
%     of the tracked object. These may become inputs in later iterations of
%     this code.
% INPUT: 
%     : {}
% OUTPUT:
%     : {}

%% Examples
% [] = planar_tracking_with_four_drones()

%%
clear all; close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
Y1 = @(t) sin(t);
Y2 = @(t) cos(t);
% Y1 = @(t) 0;
% Y2 = @(t) 0;
target_pos_vec = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
drone_pos_array = repmat(target_pos_vec,4,1) + randn(4,2);

% Fixed pos. initialisation.
% drone_pos_array = [-3 -1 1 3; 
%                   -2 -2 -2 -2]'; 

% Stationary initialisation.
drone_vel_array = zeros(4,2);  

% Random vel. initialisation.
% drone_vel_array = randn(4,2);                

% If wanted, plot the initial positions of all individuals, to check that
% there is no initial problem, i.e., no initial overlapping of drones.
if 1
    figure();
    hold on;
    scatter(drone_pos_array(:,1),drone_pos_array(:,2), 100, 'g');
    quiver(drone_pos_array(:,1),drone_pos_array(:,2),...
           drone_vel_array(:,1),drone_vel_array(:,2), 'k')
    scatter(target_pos_vec(1), target_pos_vec(2), 100, 'rx');
    shg;
end

% Each drone will have an estimate of the position of the tracked target.
% Currently this is just written is as a noisy measurement, but it can be
% written as a functional form.
est_pos_array = repmat(target_pos_vec,4,1) + 0.1*randn(4,2);

% Plot the estimated positions if necessary.
if 0
    scatter(est_pos_array(:,1), est_pos_array(:,2), 100, 'g');
end

% Compute all unit vectors.
[r_unit_direction_array] = direction_finder(drone_pos_array);
[v_unit_orientation_array] = orientation_finder(drone_vel_array);
[y_unit_target_dir_array] = target_finder(drone_pos_array,...
                                          target_pos_vec);

%%

% Choose a time-length and number of time-steps, to then calculate the
% entire deterministic trajectory of the target.
S = 0;
L = 100;
T = 5001;
dt = (L-S)/(T-1);
all_time = linspace(S,L,T);
target_trajectory = [Y1(all_time);Y2(all_time)]';

% Control parameters.
alpha = 10;
beta = 0.5;
v_sum = sum(v_unit_orientation_array,1);

% Define the functional form of dV/dt = F (Eq. 21) with the argument being 
% the drone being calculated.
F = @(i) alpha*y_unit_target_dir_array(i,:) - beta*v_sum + ...
         beta*v_unit_orientation_array(i,:) - drone_vel_array(i,:);

%%

% Create new arrays to hold all previous trajectory points.
drone_trajectory_array = zeros(T,8);
drone_dir_array = zeros(T,8);
drone_trajectory_array(1,:) = reshape(drone_pos_array',1,8);
drone_dir_array(1,:) = reshape(drone_vel_array',1,8);
drone_pos_previous = drone_trajectory_array(1,:);
drone_vel_previous = drone_dir_array(1,:);

% Set up the animation to view the trajectories.
full_traj = figure();

g = zeros(1,5);
for i = 1:5
    g(i) = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
end

% g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
% g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
% g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
% g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
% g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');

% Make animated line shorter to make it easier to view.
if 1
    h1 = animatedline('Color','r','MaximumNumPoints',100);
    h2 = animatedline('Color','b','MaximumNumPoints',100);
    h3 = animatedline('Color','g','MaximumNumPoints',100);
    h4 = animatedline('Color','m','MaximumNumPoints',100);
    h5 = animatedline('Color','k','MaximumNumPoints',100);
else
    h1 = animatedline('Color','r');
    h2 = animatedline('Color','b');
    h3 = animatedline('Color','g');
    h4 = animatedline('Color','m');
    h5 = animatedline('Color','k');
end
axis([-3,3,-3,3]);
legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
shg;

v_sum_vec = zeros(T,2);

for t = 2:T
    
    % Reshape the arrays.
    drone_pos_array = reshape(drone_pos_previous,2,4)';
    drone_vel_array = reshape(drone_vel_previous,2,4)';
    target_pos_vec = [Y1(all_time(t)),Y2(all_time(t))];

    % Recompute all unit vectors.
    [r_unit_direction_array] = direction_finder(drone_pos_array);
    [v_unit_orientation_array] = orientation_finder(drone_vel_array);
    [y_unit_target_dir_array] = target_finder(drone_pos_array,...
                                              target_pos_vec);
    v_sum = sum(v_unit_orientation_array,1);
    v_sum_vec(t,:) = v_sum;
    
    % Update the drone trajectories.
    drone_trajectory_array(t,:) = drone_pos_previous + ...
                                  drone_vel_previous*dt;
    drone_pos_previous = drone_trajectory_array(t,:);  
    
    % Update the drone velocities.
    drone_dir_array(t,:) = drone_vel_previous + ...
        (alpha*reshape(y_unit_target_dir_array',1,8) - ...
        beta*repmat(v_sum,1,4) + ...
        beta*reshape(v_unit_orientation_array',1,8) - ...
        drone_vel_previous)*dt;
    drone_vel_previous = drone_dir_array(t,:);
    
    % Update the drone positions.
    for i = 1:5
        addpoints(g(i+1),drone_trajectory_array(t,i),drone_trajectory_array(t,i+1));
    end
    
%     addpoints(g1,target_trajectory(t,1),target_trajectory(t,2));
%     addpoints(g2,drone_trajectory_array(t,1),drone_trajectory_array(t,2));
%     addpoints(g3,drone_trajectory_array(t,3),drone_trajectory_array(t,4));
%     addpoints(g4,drone_trajectory_array(t,5),drone_trajectory_array(t,6));
%     addpoints(g5,drone_trajectory_array(t,7),drone_trajectory_array(t,8));
    
    % Add new points to the trajectory lines.
    addpoints(h1,target_trajectory(t,1),target_trajectory(t,2));
    addpoints(h2,drone_trajectory_array(t,1),drone_trajectory_array(t,2));
    addpoints(h3,drone_trajectory_array(t,3),drone_trajectory_array(t,4));
    addpoints(h4,drone_trajectory_array(t,5),drone_trajectory_array(t,6));
    addpoints(h5,drone_trajectory_array(t,7),drone_trajectory_array(t,8));
    
    drawnow;
    
end

%%
norm_sum_vec = zeros(1,T);
for i = 1:T
    norm_sum_vec(i) = norm(v_sum_vec(i,:));
end
figure();
plot(norm_sum_vec);
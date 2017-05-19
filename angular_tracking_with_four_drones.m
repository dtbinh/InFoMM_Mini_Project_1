function [] = angular_tracking_with_four_drones()

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

%% Example
% [] = angular_tracking_with_four_drones()

%% INCOMPLETE

%%
clear all; close all; clc; format compact;

% Functional initialisation of the target's movement.
Y1 = @(t) sin(t);
Y2 = @(t) cos(t);
T_pos_vec = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target. Note that we use the
% initials 'D' and 'T' for 'Drone' and 'Target' respectively.
D_pos_array = repmat(T_pos_vec,4,1) + randn(4,2);

% Stationary initialisation, i.e., all velocities are zero.
D_vel_array = zeros(4,2);  

% Compute all unit vectors {r,v,y} defined in the original formulation.
[r_unit_array] = direction_finder(D_pos_array);
[v_unit_array] = orientation_finder(D_vel_array);
[y_unit_array] = target_finder(D_pos_array,T_pos_vec);

% If needed, plot the initial positions of all individuals, to check that
% there is no initial problem, i.e., no initial overlapping of drones.
if 1
    figure();
    hold on;
    
    % Plot the locations of the drones and the target.
    scatter(D_pos_array(:,1),D_pos_array(:,2), 100, 'g');
    quiver(D_pos_array(:,1),D_pos_array(:,2),...
           D_vel_array(:,1),D_vel_array(:,2), 'b')
    scatter(T_pos_vec(1), T_pos_vec(2), 100, 'rx');
    
    % Plot the direction vectors from drones to all other objects.
    for i = 1:4
        quiver(repmat(D_pos_array(i,1),4,1),repmat(D_pos_array(i,2),4,1),...
            -.3*r_unit_array(:,2*i-1),-.3*r_unit_array(:,2*i), 'k');
        quiver(D_pos_array(i,1),D_pos_array(i,2),...
            .3*y_unit_array(i,1),.3*y_unit_array(i,2),'r');
    end
    
    axis equal;
    shg;
end
        
%%
% Find directions of each drone pointing to the target, and find their
% bearings (from NORTH).
D_dir_array = T_pos_vec - D_pos_array;
D_ang_array = atan(D_dir_array(:,2)./D_dir_array(:,1));
D_ang_array = D_ang_array - pi/2;

% Change the angles into degrees if necessary (mainly used to check).
D_deg_array = D_ang_array./(2*pi) * 360;

% Note that Matlab measures from NORTH by default, rather than from the
% x-axis of the complex plane. Thus, we make sure that we are working with
% positive angles to measure the true bearing.
for i = 1:length(D_deg_array)
    if D_deg_array(i) < 0
    D_deg_array(i) = D_deg_array(i) + 360;
    end
end
for i = 1:length(D_deg_array)
    d1 = D_pos_array(i,:);
    if d1(1) > T_pos_vec(1);
        D_deg_array(i) = D_deg_array(i) - 180;
    end
end

%%

% Find the mean direction.
mean_deg = mean(D_deg_array);

% Remove each current direction contribution from the mean.
mean_diff = repmat(mean_deg,4,1) - D_deg_array/4;

S = 0;
L = 100;
T = 5001;
dt = (L-S)/(T-1);
all_time = linspace(S,L,T);
target_trajectory = [Y1(all_time);Y2(all_time)]';

%%

% Create new arrays to hold all previous trajectory points.
drone_trajectory_array = zeros(T,8);
D_dir_array = zeros(T,8);
drone_trajectory_array(1,:) = reshape(D_pos_array',1,8);
D_dir_array(1,:) = reshape(D_vel_array',1,8);
drone_pos_previous = drone_trajectory_array(1,:);
drone_vel_previous = D_dir_array(1,:);

% Set up the animation to view the trajectories.
full_traj = figure();
g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');

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
axis([-3,3,-3,3]); axis equal;
legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
shg;

for t = 2:T
    
    % Reshape the arrays.
    D_pos_array = reshape(drone_pos_previous,2,4)';
    T_pos_vec = [Y1(all_time(t)),Y2(all_time(t))];
    D_dir_array = T_pos_vec - D_pos_array;
    D_ang_array = atan(D_dir_array(:,2)./D_dir_array(:,1));
    D_ang_array = D_ang_array - pi/2;
    D_deg_array = D_ang_array./(2*pi) * 360;
    
    for i = 1:length(D_deg_array)
        if D_deg_array(i) < 0
        D_deg_array(i) = D_deg_array(i) + 360;
        end
    end
    for i = 1:length(D_deg_array)
        d1 = D_pos_array(i,:);
        if d1(1) > T_pos_vec(1);
            D_deg_array(i) = D_deg_array(i) - 180;
        end
    end

    % Recompute all unit vectors.
    r_unit_array = direction_finder(D_pos_array);
    r_angle_array = relative_bearing(r_unit_array);
    y_unit_array = target_finder(D_pos_array,...
                                              T_pos_vec);

    % Find the average direction to all other drones, to try and centralise
    % them.
    r_sum = sum(r_angle_array,2)
    
                                          
    % Update the drone trajectories.
    drone_trajectory_array(t,:) = drone_pos_previous + ...
                                  drone_vel_previous*dt;
    drone_pos_previous = drone_trajectory_array(t,:);  
    
    % Update the drone velocities.
    D_dir_array(t,:) = drone_vel_previous + ...
        (alpha*reshape(y_unit_array',1,8) - ...
        beta*repmat(v_sum,1,4) + ...
        beta*reshape(v_unit_array',1,8) - ...
        drone_vel_previous)*dt;
    drone_vel_previous = D_dir_array(t,:);
    
    % Update the drone positions.
    addpoints(g1,target_trajectory(t,1),target_trajectory(t,2));
    addpoints(g2,drone_trajectory_array(t,1),drone_trajectory_array(t,2));
    addpoints(g3,drone_trajectory_array(t,3),drone_trajectory_array(t,4));
    addpoints(g4,drone_trajectory_array(t,5),drone_trajectory_array(t,6));
    addpoints(g5,drone_trajectory_array(t,7),drone_trajectory_array(t,8));
    
    % Add new points to the trajectory lines.
    addpoints(h1,target_trajectory(t,1),target_trajectory(t,2));
    addpoints(h2,drone_trajectory_array(t,1),drone_trajectory_array(t,2));
    addpoints(h3,drone_trajectory_array(t,3),drone_trajectory_array(t,4));
    addpoints(h4,drone_trajectory_array(t,5),drone_trajectory_array(t,6));
    addpoints(h5,drone_trajectory_array(t,7),drone_trajectory_array(t,8));
    
    drawnow;
    
end


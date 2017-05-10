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

%% Examples
% [] = angular_tracking_with_four_drones()

%%
clear all; close all; clc; format compact;

% Functional initialisation of the target's movement.
Y1 = @(t) sin(t);
Y2 = @(t) cos(t);
target_pos_vec = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
drone_pos_array = repmat(target_pos_vec,4,1) + randn(4,2);

% Stationary initialisation.
drone_vel_array = zeros(4,2);  

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

% Compute all unit vectors.
[r_unit_direction_array] = direction_finder(drone_pos_array);
[v_unit_orientation_array] = orientation_finder(drone_vel_array);
[y_unit_target_dir_array] = target_finder(drone_pos_array,...
                                          target_pos_vec);

for i = 1:4
    quiver(repmat(drone_pos_array(i,1),4,1),repmat(drone_pos_array(i,2),4,1),...
    -.3*r_unit_direction_array(:,2*i-1),-.3*r_unit_direction_array(:,2*i), 'k');
    quiver(drone_pos_array(i,1),drone_pos_array(i,2),...
    .3*y_unit_target_dir_array(i,1),.3*y_unit_target_dir_array(i,2),'r');
end    
shg;
        
%%
% Find directions of each drone pointing to the target, and find their
% bearings (from NORTH).
drone_dir_array = target_pos_vec - drone_pos_array;
drone_angles_array = atan(drone_dir_array(:,2)./drone_dir_array(:,1));
drone_angles_array = drone_angles_array - pi/2;
drone_degrees_array = drone_angles_array./(2*pi) * 360;

for i = 1:length(drone_degrees_array)
    if drone_degrees_array(i) < 0
    drone_degrees_array(i) = drone_degrees_array(i) + 360;
    end
end
for i = 1:length(drone_degrees_array)
    d1 = drone_pos_array(i,:);
    if d1(1) > target_pos_vec(1);
        drone_degrees_array(i) = drone_degrees_array(i) - 180;
    end
end

%%
deg_diff_array = zeros(4,4);
for i = 1:4
    for j = 1:4
        deg_diff_array(i,j) = drone_degrees_array(i) - drone_degrees_array(j);
    end
end




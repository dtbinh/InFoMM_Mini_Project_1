function [] = planar_tracking_with_four_drones(L,T,a,b)

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
%     L: {float} Length of the simulation (seconds).
%     T: {int} Number of data points wanted in the simulation.
%     a: {float} 'alpha' in the original system equations.
%     b: {float} 'beta' in the original system equations.
% OUTPUT:
%     : {}

%% Examples
% planar_tracking_with_four_drones(100,5001,1.5,0.5)

%%
keepvars = {'L','T','a','b'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
Y1 = @(t) sin(t);
Y2 = @(t) cos(t);
T_pos_vec = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
D_pos_array = repmat(T_pos_vec,4,1) + randn(4,2);

% Fixed pos. initialisation.
% D_pos_array = [-3 -1 1 3; 
%                  -2 -2 -2 -2]'; 

% Stationary initialisation.
D_vel_array = zeros(4,2);  

% Random vel. initialisation.
% D_vel_array = randn(4,2);    

% Compute all unit vectors {r,v,y} defined in the original formulation.
[r_unit_array] = direction_finder(D_pos_array);
[v_unit_array] = orientation_finder(D_vel_array);
[y_unit_array] = target_finder(D_pos_array,T_pos_vec);

% If wanted, plot the initial positions of all individuals, to check that
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
    end    
    
    axis equal;
    shg;
end

%%

% Compute the time-step, then calculate the entire deterministic trajectory
% of the target.
dt = L/(T-1);
all_time = linspace(0,L,T);
T_trajectory = [Y1(all_time);Y2(all_time)]';

% Control parameters.
alpha = a;
beta = b;
v_repulsion = sum(v_unit_array,1);

% Define the functional form of dV/dt = F (Eq. 21) with the argument being 
% the drone being calculated.
F = @(i) alpha*y_unit_array(i,:) - beta*v_repulsion + ...
         beta*v_unit_array(i,:) - D_vel_array(i,:);

%%

% Create new arrays to hold all previous trajectory points.
D_traj_array = zeros(T,8);
D_dir_array = zeros(T,8);
D_traj_array(1,:) = reshape(D_pos_array',1,8);
D_dir_array(1,:) = reshape(D_vel_array',1,8);
D_pos_previous = D_traj_array(1,:);
D_vel_previous = D_dir_array(1,:);

% Set up the animation to view the trajectories.
traj_plot = figure();
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
axis([-3,3,-3,3]);
legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
shg;

v_sum_vec = zeros(T,2);

for t = 2:T
    
    % Reshape the arrays.
    D_pos_array = reshape(D_pos_previous,2,4)';
    D_vel_array = reshape(D_vel_previous,2,4)';
    T_pos_vec = [Y1(all_time(t)),Y2(all_time(t))];

    % Recompute all unit vectors.
    [r_unit_array] = direction_finder(D_pos_array);
    [v_unit_array] = orientation_finder(D_vel_array);
    [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
    v_repulsion = sum(v_unit_array,1);
    v_sum_vec(t,:) = v_repulsion;
    
    % Update the drone trajectories.
    D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
    D_pos_previous = D_traj_array(t,:);  
    
    % Update the drone velocities.
    D_dir_array(t,:) = D_vel_previous + ...
        (alpha*reshape(y_unit_array',1,8) - ...
        beta*repmat(v_repulsion,1,4) + ...
        beta*reshape(v_unit_array',1,8) - ...
        D_vel_previous)*dt;
    D_vel_previous = D_dir_array(t,:);
    
    % Update the drone positions.
    addpoints(g1,T_trajectory(t,1),T_trajectory(t,2));
    addpoints(g2,D_traj_array(t,1),D_traj_array(t,2));
    addpoints(g3,D_traj_array(t,3),D_traj_array(t,4));
    addpoints(g4,D_traj_array(t,5),D_traj_array(t,6));
    addpoints(g5,D_traj_array(t,7),D_traj_array(t,8));
    
    % Add new points to the trajectory lines.
    addpoints(h1,T_trajectory(t,1),T_trajectory(t,2));
    addpoints(h2,D_traj_array(t,1),D_traj_array(t,2));
    addpoints(h3,D_traj_array(t,3),D_traj_array(t,4));
    addpoints(h4,D_traj_array(t,5),D_traj_array(t,6));
    addpoints(h5,D_traj_array(t,7),D_traj_array(t,8));
    
    drawnow;
    
end

%%

% If we want, we can also plot the magnitude of the repulsion vector over
% time, to see how/if it balances within the system.
norm_sum_vec = zeros(1,T);
for i = 1:T
    norm_sum_vec(i) = norm(v_sum_vec(i,:));
end
figure();
plot(norm_sum_vec);
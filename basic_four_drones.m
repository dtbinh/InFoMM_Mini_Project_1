function [] = basic_four_drones(L,T,a,b)

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
%      : {}

%% Example
% [] = basic_four_drones(100,5001,1.5,0.5)
% [] = basic_four_drones(100,5001,10,10)
% [] = basic_four_drones(800,20001,10,10)

%%
keepvars = {'L','T','a','b'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
% Y1 = @(t) 5*sin(t/15);
% Y2 = @(t) 5*cos(t/15);
Y1 = @(t) 0*sin(t/15);
Y2 = @(t) 0*cos(t/15);
tar_pos_V = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
dro_pos_A = repmat(tar_pos_V,4,1) + randn(4,2);

% Stationary initialisation.
dro_vel_A = zeros(4,2);  

% Random vel. initialisation.
% dro_vel_A = randn(4,2);   

% Circular orbit initialisation.
% y_unit_A = target_finder(dro_pos_A,tar_pos_V);
% dro_vel_A = ([0, -1; 1 0]*y_unit_A')';

% Compute all unit vectors {r,v,y} defined in the original formulation.
r_unit_A = direction_finder(dro_pos_A);
v_unit_A = orientation_finder(dro_vel_A);
y_unit_A = target_finder(dro_pos_A,tar_pos_V);

% If wanted, plot the initial positions of all individuals, to check that
% there is no initial problem, i.e., no initial overlapping of drones.
if 0
    figure();
    hold on;
    
    % Plot the locations of the drones and the target.
    scatter(dro_pos_A(:,1),dro_pos_A(:,2), 100, 'g');
    quiver(dro_pos_A(:,1),dro_pos_A(:,2),...
           dro_vel_A(:,1),dro_vel_A(:,2), 'b')
    scatter(T_pos_vec(1), T_pos_vec(2), 100, 'rx');
    
    % Plot the direction vectors from drones to all other objects.
    for i = 1:4
        quiver(repmat(dro_pos_A(i,1),4,1),repmat(dro_pos_A(i,2),4,1),...
        -.3*r_unit_A(:,2*i-1),-.3*r_unit_A(:,2*i), 'k');
    end    
    
    axis square;
    shg;
end

% Compute the time-step, then calculate the entire deterministic trajectory
% of the target.
dt = L/(T-1);
all_time_V = linspace(0,L,T);
tar_traj_A = [Y1(all_time_V);Y2(all_time_V)]';

% Control parameters.
alpha = a;
beta = b;
v_repulsion_V = sum(v_unit_A,1);

% Create new arrays to hold all previous trajectory points.
dro_traj_A = zeros(T,8);
dro_direc_A = zeros(T,8);
dro_traj_A(1,:) = reshape(dro_pos_A',1,8);
dro_direc_A(1,:) = reshape(dro_vel_A',1,8);
dro_pos_prev_V = dro_traj_A(1,:);
dro_vel_prev_V = dro_direc_A(1,:);

% Set up the animation to view the trajectories.
trajectories_plot = figure();
g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');

% Make animated line shorter to make it easier to view.
h1 = animatedline('Color','r','MaximumNumPoints',100);
h2 = animatedline('Color','b','MaximumNumPoints',100);
h3 = animatedline('Color','g','MaximumNumPoints',100);
h4 = animatedline('Color','m','MaximumNumPoints',100);
h5 = animatedline('Color','k','MaximumNumPoints',100);
axis([-1,1,-1,1]);
legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
shg;

v_sum_V = zeros(T,2);

for t = 2:T
    
    % Reshape the arrays.
    dro_pos_A = reshape(dro_pos_prev_V,2,4)';
    dro_vel_A = reshape(dro_vel_prev_V,2,4)';
    tar_pos_V = [Y1(all_time_V(t)),Y2(all_time_V(t))];

    % Recompute all unit vectors.
    r_unit_A = direction_finder(dro_pos_A);
    v_unit_A = orientation_finder(dro_vel_A);
    y_unit_A = target_finder(dro_pos_A,tar_pos_V);
    v_repulsion_V = sum(v_unit_A,1);
    v_sum_V(t,:) = v_repulsion_V;
    
    % Update the drone trajectories.
    dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
    dro_pos_prev_V = dro_traj_A(t,:);  
    
    % Update the drone velocities.
    dro_direc_A(t,:) = dro_vel_prev_V + ...
        (alpha*reshape(y_unit_A',1,8) - ...
        beta*repmat(v_repulsion_V,1,4) + ...
        beta*reshape(v_unit_A',1,8) - ...
        dro_vel_prev_V)*dt;
    dro_vel_prev_V = dro_direc_A(t,:);
    
    % Update the drone positions.
    addpoints(g1,tar_traj_A(t,1),tar_traj_A(t,2));
    addpoints(g2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(g3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(g4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(g5,dro_traj_A(t,7),dro_traj_A(t,8));
    
    % Add new points to the trajectory lines.
    addpoints(h1,tar_traj_A(t,1),tar_traj_A(t,2));
    addpoints(h2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(h3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(h4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(h5,dro_traj_A(t,7),dro_traj_A(t,8));
    
    drawnow;
    
end

end
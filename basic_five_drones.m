function [] = basic_five_drones(L,T,a,b)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     five individual drones and a single object to be tracked. In this
%     formulation, we will impose all parameters, as well as the movement
%     of the tracked object.
% INPUT: 
%     L: {float} Length of the simulation.
%     T:   {int} Number of data points wanted in the simulation.
%     a: {float} 'alpha' in the original system equations.
%     b: {float} 'beta' in the original system equations.
% OUTPUT:
%      : {}

%% Example
% basic_five_drones(100,5001,10,10)

%%
keepvars = {'L','T','a','b'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
Y1 = @(t) 0*sin(t/15);
Y2 = @(t) 0*cos(t/15);
tar_pos_V = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
dro_pos_A = repmat(tar_pos_V,5,1) + randn(5,2);

% Stationary initialisation.
dro_vel_A = zeros(5,2);  

% Random vel. initialisation.
dro_vel_A = randn(5,2);   

% Circular orbit initialisation.
% y_unit_A = target_finder(dro_pos_A,tar_pos_V);
% dro_vel_A = ([0, -1; 1 0]*y_unit_A')';
% % Send one the wrong way.
% dro_vel_A(5,:) = -dro_vel_A(5,:);

% Random Strength Circular orbit initialisation.
% y_unit_A = target_finder(dro_pos_A,tar_pos_V);
% dro_vel_A = 15*diag(diag(abs(randn(5))))*([0, -1; 1 0]*y_unit_A')'



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
    for i = 1:5
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
dro_traj_A = zeros(T,10);
dro_direc_A = zeros(T,10);
dro_traj_A(1,:) = reshape(dro_pos_A',1,10);
dro_direc_A(1,:) = reshape(dro_vel_A',1,10);
dro_pos_prev_V = dro_traj_A(1,:);
dro_vel_prev_V = dro_direc_A(1,:);

% Set up the animation to view the trajectories.
trajectories_plot = figure();
g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
g6 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');

% Make animated line shorter to make it easier to view.
h1 = animatedline('Color','r','MaximumNumPoints',100);
h2 = animatedline('Color','b','MaximumNumPoints',100);
h3 = animatedline('Color','g','MaximumNumPoints',100);
h4 = animatedline('Color','m','MaximumNumPoints',100);
h5 = animatedline('Color','k','MaximumNumPoints',1000);
h6 = animatedline('Color','r','MaximumNumPoints',1000);
axis([-20,20,-20,20]);
legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5')
shg;

v_sum_V = zeros(T,2);
vel_mag_V = zeros(T,5);

for t = 2:T
    
    if mod(t,200) == 0
        % Time counter, to ensure the code is running.
        t_count = t
    end
    
    % Reshape the arrays.
    dro_pos_A = reshape(dro_pos_prev_V,2,5)';
    dro_vel_A = reshape(dro_vel_prev_V,2,5)';
    tar_pos_V = [Y1(all_time_V(t)),Y2(all_time_V(t))];
    
    % Compute the velocity magnitudes, to plot convergence.
    for i = 1:5
        vel_mag_V(t,i) = norm(dro_vel_A(i,:));
    end

    % Recompute all unit vectors.
    r_unit_A = direction_finder(dro_pos_A);
    v_unit_A = orientation_finder(dro_vel_A);
    y_unit_A = target_finder(dro_pos_A,tar_pos_V);
    v_repulsion_V = sum(v_unit_A,1);
    v_sum_V(t,:) = norm(v_repulsion_V);
    
    % Update the drone trajectories.
    dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
    dro_pos_prev_V = dro_traj_A(t,:);  
    
    % Update the drone velocities.
    dro_direc_A(t,:) = dro_vel_prev_V + ...
        (alpha*reshape(y_unit_A',1,10) - ...
        beta*repmat(v_repulsion_V,1,5) + ...
        beta*reshape(v_unit_A',1,10) - ...
        dro_vel_prev_V)*dt;
    dro_vel_prev_V = dro_direc_A(t,:);
    
    % Update the drone positions.
    addpoints(g1,tar_traj_A(t,1),tar_traj_A(t,2));
    addpoints(g2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(g3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(g4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(g5,dro_traj_A(t,7),dro_traj_A(t,8));
    addpoints(g6,dro_traj_A(t,9),dro_traj_A(t,10));
    
    % Add new points to the trajectory lines.
    addpoints(h1,tar_traj_A(t,1),tar_traj_A(t,2));
    addpoints(h2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(h3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(h4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(h5,dro_traj_A(t,7),dro_traj_A(t,8));
    addpoints(h6,dro_traj_A(t,9),dro_traj_A(t,10));
    
    drawnow;
    
end

VELS = subplot(1,2,1);
hold on;
plot(vel_mag_V(:,1));
plot(vel_mag_V(:,2));
plot(vel_mag_V(:,3));
plot(vel_mag_V(:,4));
plot(vel_mag_V(:,5));
shg;
xlabel('Time','interpreter','latex');
ylabel('Velocity','interpreter','latex');
legend('Drone 1','Drone 2','Drone 3','Drone 4','Drone 5');

VELSUM = subplot(1,2,2);
plot(v_sum_V);
xlabel('Time','interpreter','latex');
ylabel('$||V_{sum}||$','interpreter','latex');
shg;

end
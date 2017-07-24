function [] = basic_twelve_drones(L,T,a,b)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     twelve individual drones and a single object to be tracked. In this
%     formulation, we will impose all parameters, as well as the movement
%     of the tracked object.
% INPUT: 
%     L: {float} Length of the simulation (seconds).
%     T:   {int} Number of data points wanted in the simulation.
%     a: {float} 'alpha' in the original system equations.
%     b: {float} 'beta' in the original system equations.
% OUTPUT:
%      : {}

%% Example
% basic_twelve_drones(100,5001,10,10)

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
dro_pos_A = repmat(tar_pos_V,12,1) + 2*randn(12,2);

% Stationary initialisation.
% dro_vel_A = zeros(12,2);  

% Random vel. initialisation.
% dro_vel_A = randn(12,2);   

% Circular orbit initialisation.
y_unit_A = target_finder(dro_pos_A,tar_pos_V);
dro_vel_A = 5*([0, -1; 1 0]*y_unit_A')';
% Send one the wrong way.
% dro_vel_A(12,:) = -dro_vel_A(12,:);

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
        quiver(repmat(dro_pos_A(i,1),12,1),repmat(dro_pos_A(i,2),12,1),...
        -.3*r_unit_A(:,2*i-1),-.3*r_unit_A(:,2*i), 'k');
    end    
    
    axis square;
    shg;
end

% Compute the time-step, then calculate the entire deterministic trajectory
% of the target.
dt = L/(T-1);
all_time_V = linspace(0,L,T);
tar_trajectory_A = [Y1(all_time_V);Y2(all_time_V)]';

% Control parameters.
alpha = a;
beta = b;
v_repulsion_V = sum(v_unit_A,1);

% Create new arrays to hold all previous trajectory points.
dro_traj_A = zeros(T,24);
dro_direc_A = zeros(T,24);
dro_traj_A(1,:) = reshape(dro_pos_A',1,24);
dro_direc_A(1,:) = reshape(dro_vel_A',1,24);
dro_pos_prev_V = dro_traj_A(1,:);
dro_vel_prev_V = dro_direc_A(1,:);

% Set up the animation to view the trajectories.
trajectories_plot = figure();
g1a = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
g1b = animatedline('Color','r','MaximumNumpoints',1,'Marker','o');
g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o','MarkerSize',10);
g3 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o','MarkerSize',10);
g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o','MarkerSize',10);
g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o','MarkerSize',10);
g6 = animatedline('Color','b','MaximumNumPoints',1,'Marker','s','MarkerSize',10);
g7 = animatedline('Color','r','MaximumNumPoints',1,'Marker','s','MarkerSize',10);
g8 = animatedline('Color','m','MaximumNumPoints',1,'Marker','s','MarkerSize',10);
g9 = animatedline('Color','k','MaximumNumPoints',1,'Marker','s','MarkerSize',10);
g10 = animatedline('Color','b','MaximumNumPoints',1,'Marker','d','MarkerSize',10);
g11 = animatedline('Color','r','MaximumNumPoints',1,'Marker','d','MarkerSize',10);
g12 = animatedline('Color','m','MaximumNumPoints',1,'Marker','d','MarkerSize',10);
g13 = animatedline('Color','k','MaximumNumPoints',1,'Marker','d','MarkerSize',10);

% Make animated line shorter to make it easier to view.
h1 = animatedline('Color','r','MaximumNumPoints',20);
h2 = animatedline('Color','b','MaximumNumPoints',20);
h3 = animatedline('Color','r','MaximumNumPoints',20);
h4 = animatedline('Color','m','MaximumNumPoints',20);
h5 = animatedline('Color','k','MaximumNumPoints',20);
h6 = animatedline('Color','b','MaximumNumPoints',20,'LineStyle','-.');
h7 = animatedline('Color','r','MaximumNumPoints',20,'LineStyle','-.');
h8 = animatedline('Color','m','MaximumNumPoints',20,'LineStyle','-.');
h9 = animatedline('Color','k','MaximumNumPoints',20,'LineStyle','-.');
h10 = animatedline('Color','b','MaximumNumPoints',20,'LineStyle','--');
h11 = animatedline('Color','r','MaximumNumPoints',20,'LineStyle','--');
h12 = animatedline('Color','m','MaximumNumPoints',20,'LineStyle','-');
h13 = animatedline('Color','k','MaximumNumPoints',20,'LineStyle','-');
axis([-20 20 -20 20]);
legend('Target','Centroid','Drone 1','Drone 2','Drone 3','Drone 4',...
    'Drone 5','Drone 6','Drone 7','Drone 8','Drone 9','Drone 10',...
    'Drone 11','Drone 12');
shg;

v_sum_V = zeros(T,2);
ang_mom_V = zeros(T,2);
vel_mag_V = zeros(T,12);

for t = 2:T
    
    if mod(t,200) == 0
        % Time counter, to ensure the code is running.
        t_count = t
    end
    
    % Reshape the arrays.
    dro_pos_A = reshape(dro_pos_prev_V,2,12)';
    dro_vel_A = reshape(dro_vel_prev_V,2,12)';
    tar_pos_V = [Y1(all_time_V(t)),Y2(all_time_V(t))];
    
    % Compute the velocity magnitudes, to plot convergence.
    for i = 1:12
        vel_mag_V(t,i) = norm(dro_vel_A(i,:));
    end

    % Recompute all unit vectors.
    r_unit_A = direction_finder(dro_pos_A);
    v_unit_A = orientation_finder(dro_vel_A);
    y_unit_A = target_finder(dro_pos_A,tar_pos_V);
    v_repulsion_V = sum(v_unit_A,1);
    v_sum_V(t,:) = v_repulsion_V;
    
    % Compute the angular momentum, to see if it is conserved at all times,
    % or only in the final solution.
    L = angular_momentum(dro_vel_A,dro_pos_A,y_unit_A,12);
    ang_mom_V(t) = L;
    
    % Update the drone trajectories.
    dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
    dro_pos_prev_V = dro_traj_A(t,:);
    centroid_V = sum([dro_pos_prev_V(1:2);
    dro_pos_prev_V(3:4);
    dro_pos_prev_V(5:6);
    dro_pos_prev_V(7:8);
    dro_pos_prev_V(9:10);
    dro_pos_prev_V(11:12);
    dro_pos_prev_V(13:14);
    dro_pos_prev_V(15:16);
    dro_pos_prev_V(17:18);
    dro_pos_prev_V(19:20);
    dro_pos_prev_V(21:22);
    dro_pos_prev_V(23:24)],1)/12;
    
    % Update the drone velocities.
    dro_direc_A(t,:) = dro_vel_prev_V + ...
        (alpha*reshape(y_unit_A',1,24) - ...
        beta*repmat(v_repulsion_V,1,12) + ...
        beta*reshape(v_unit_A',1,24) - ...
        dro_vel_prev_V)*dt;
    dro_vel_prev_V = dro_direc_A(t,:);
    
    % Update the drone positions.
    addpoints(g1a,tar_trajectory_A(t,1),tar_trajectory_A(t,2));
    addpoints(g1b,centroid_V(1),centroid_V(2));
    addpoints(g2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(g3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(g4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(g5,dro_traj_A(t,7),dro_traj_A(t,8));
    addpoints(g6,dro_traj_A(t,9),dro_traj_A(t,10));
    addpoints(g7,dro_traj_A(t,11),dro_traj_A(t,12));
    addpoints(g8,dro_traj_A(t,13),dro_traj_A(t,14));
    addpoints(g9,dro_traj_A(t,15),dro_traj_A(t,16));
    addpoints(g10,dro_traj_A(t,17),dro_traj_A(t,18));
    addpoints(g11,dro_traj_A(t,19),dro_traj_A(t,20));
    addpoints(g12,dro_traj_A(t,21),dro_traj_A(t,22));
    addpoints(g13,dro_traj_A(t,23),dro_traj_A(t,24));
    
    % Add new points to the trajectory lines.
    addpoints(h1,tar_trajectory_A(t,1),tar_trajectory_A(t,2));
    addpoints(h2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(h3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(h4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(h5,dro_traj_A(t,7),dro_traj_A(t,8));
    addpoints(h6,dro_traj_A(t,9),dro_traj_A(t,10));
    addpoints(h7,dro_traj_A(t,11),dro_traj_A(t,12));
    addpoints(h8,dro_traj_A(t,13),dro_traj_A(t,14));
    addpoints(h9,dro_traj_A(t,15),dro_traj_A(t,16));
    addpoints(h10,dro_traj_A(t,17),dro_traj_A(t,18));
    addpoints(h11,dro_traj_A(t,19),dro_traj_A(t,20));
    addpoints(h12,dro_traj_A(t,21),dro_traj_A(t,22));
    addpoints(h13,dro_traj_A(t,23),dro_traj_A(t,24));
    
    drawnow;
    
end

% Plot the behaviour of the angular momentum of the system.
figure();
plot(ang_mom_V);
shg;

figure();
hold on;
plot(vel_mag_V(:,1));
plot(vel_mag_V(:,2));
plot(vel_mag_V(:,3));
plot(vel_mag_V(:,4));
plot(vel_mag_V(:,5));
plot(vel_mag_V(:,6));
plot(vel_mag_V(:,7));
plot(vel_mag_V(:,8));
plot(vel_mag_V(:,9));
plot(vel_mag_V(:,10));
plot(vel_mag_V(:,11));
plot(vel_mag_V(:,12));
shg;

end
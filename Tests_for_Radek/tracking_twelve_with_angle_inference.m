function [] = tracking_twelve_with_angle_inference(L,T,a,b,g,e)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     six individual drones and a single object to be tracked. In this
%     formulation, we will impose all parameters, as well as the movement
%     of the tracked object. These may become inputs in later iterations of
%     this code. Note that this involves an amended formulation of the toy
%     problem: dV/dt = a*y - b*sum(v(j)) - b*V + g*sum(dr(ij)/dt)*r(rij).
%     This may also be changed to include numerical second derivatives too.
% INPUT: 
%     L: {float} Length of the simulation (seconds).
%     T: {int} Number of data points wanted in the simulation.
%     a: {float} 'alpha' in the original system equations.
%     b: {float} 'beta' in the original system equations.
%     g: {float} 'gamma' in the amended system equations.
%     e: {float} 'eta' in the amended system equations.
% OUTPUT:
%     : {}

%% Examples
% tracking_twelve_with_angle_inference(100,5001,10,10,10,100) w/ slow circ
% tracking_twelve_with_angle_inference(100,5001,5,5,50,100) w/ stationary

%%
keepvars = {'L','T','a','b','g','e'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
% Y1 = @(t) 10*sin(t/5);
% Y2 = @(t) 10*cos(t/5);
Y1 = @(t) 0.0001*cos(t/20);
Y2 = @(t) 0.0001*sin(t/20);
tar_pos_V = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
dro_pos_A = repmat(tar_pos_V,12,1) + 3*randn(12,2);
centroid_V = sum(dro_pos_A,1)/12;

% Stationary initialisation.
dro_vel_A = zeros(12,2);  

% Random vel. initialisation.
% dro_vel_A = randn(4,2);    

% Circular orbit initialisation.
y_unit_A = target_finder(dro_pos_A,tar_pos_V);
dro_vel_A = ([0, -pi/2; pi/2 0]*y_unit_A')';

% Compute all unit vectors {r,v,y} defined in the original formulation.
r_unit_A = direction_finder(dro_pos_A);
r_prev_A = r_unit_A;
v_unit_A = orientation_finder(dro_vel_A);
y_unit_A = target_finder(dro_pos_A,tar_pos_V);

% If wanted, plot the initial positions of all individuals, to check that
% there is no initial problem, i.e., no initial overlapping of drones.
if 1
    initial_setup = figure();
    hold on;
    
    % Plot the locations of the drones and the target.
    scatter(dro_pos_A(:,1),dro_pos_A(:,2), 100, 'g');
    scatter(centroid_V(1),centroid_V(2),100,'ro');
    quiver(dro_pos_A(:,1),dro_pos_A(:,2),...
           dro_vel_A(:,1),dro_vel_A(:,2), 'b')
    scatter(tar_pos_V(1), tar_pos_V(2), 100, 'rx');
    
    % Plot the direction vectors from drones to all other objects.
    for i = 1:12
        quiver(repmat(dro_pos_A(i,1),12,1),repmat(dro_pos_A(i,2),12,1),...
        -.3*r_unit_A(:,2*i-1),-.3*r_unit_A(:,2*i), 'k');
    end    
    axis equal;
    shg;
end

% Compute the time-step, then calculate the entire deterministic trajectory
% of the target.
dt_S = L/(T-1);
all_time_V = linspace(0,L,T);
tar_trajectory_A = [Y1(all_time_V);Y2(all_time_V)]';

% Control parameters.
alpha = a;
beta = b;
gamma = g;
eta = e;
v_repulsion_V = sum(v_unit_A,1);

% Create new arrays to hold all previous trajectory points.
dro_traj_A = zeros(T,24);
dro_direc_A = zeros(T,24);
dro_traj_A(1,:) = reshape(dro_pos_A',1,24);
dro_direc_A(1,:) = reshape(dro_vel_A',1,24);
dro_pos_prev_V = dro_traj_A(1,:);
dro_vel_prev_V = dro_direc_A(1,:);

% Set up the animation to view the trajectories.
traj_plot = figure();
g1a = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
g1b = animatedline('Color','r','MaximumNumpoints',1,'Marker','o');
g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
g6 = animatedline('Color','b','MaximumNumPoints',1,'Marker','s');
g7 = animatedline('Color','g','MaximumNumPoints',1,'Marker','s');
g8 = animatedline('Color','m','MaximumNumPoints',1,'Marker','s');
g9 = animatedline('Color','k','MaximumNumPoints',1,'Marker','s');
g10 = animatedline('Color','b','MaximumNumPoints',1,'Marker','d');
g11 = animatedline('Color','g','MaximumNumPoints',1,'Marker','d');
g12 = animatedline('Color','m','MaximumNumPoints',1,'Marker','d');
g13 = animatedline('Color','k','MaximumNumPoints',1,'Marker','d');

% Make animated line shorter to make it easier to view.
h1 = animatedline('Color','r','MaximumNumPoints',100);
h2 = animatedline('Color','b','MaximumNumPoints',100);
h3 = animatedline('Color','g','MaximumNumPoints',100);
h4 = animatedline('Color','m','MaximumNumPoints',100);
h5 = animatedline('Color','k','MaximumNumPoints',100);
h6 = animatedline('Color','b','MaximumNumPoints',100,'LineStyle','-.');
h7 = animatedline('Color','g','MaximumNumPoints',100,'LineStyle','-.');
h8 = animatedline('Color','m','MaximumNumPoints',100,'LineStyle','-.');
h9 = animatedline('Color','k','MaximumNumPoints',100,'LineStyle','-.');
h10 = animatedline('Color','b','MaximumNumPoints',100,'LineStyle','--');
h11 = animatedline('Color','g','MaximumNumPoints',100,'LineStyle','--');
h12 = animatedline('Color','m','MaximumNumPoints',100,'LineStyle','--');
h13 = animatedline('Color','k','MaximumNumPoints',100,'LineStyle','--');
axis([-40,40,-40,40]);
legend('Target','Centroid','Drone 1','Drone 2','Drone 3','Drone 4',...
    'Drone 5','Drone 6','Drone 7','Drone 8','Drone 9','Drone 10',...
    'Drone 11','Drone 12');
shg;

% Keep track of the repulsion factor.
v_sum_V = zeros(T,2);
repulsion_A = zeros(12);
y_factor_V = zeros(12,1);

for t = 2:T
    
    if mod(t,50) == 0
        % Time counter, to ensure the code is running.
        t_count_S = t
        sum(dash_magnitude_V)
    end
    
    % Reshape the arrays.
    dro_pos_A = reshape(dro_pos_prev_V,2,12)';
    dro_vel_A = reshape(dro_vel_prev_V,2,12)';
    tar_pos_V = [Y1(all_time_V(t)),Y2(all_time_V(t))];

    % Recompute all unit vectors.
    r_unit_A = direction_finder(dro_pos_A);
    
    % Compute the factor from which we shall try to avoid close distances.
    r_diff_A = r_prev_A - r_unit_A;
    r_factor_A = zeros(12);
    for i = 1:11
        for j = (i+1):12
            r_factor_A(i,j) = ...
            1*norm([r_diff_A(i,2*j-1),r_diff_A(i,2*j)]);
        end
    end
    r_factor_A = r_factor_A + r_factor_A';
    r_prev_A = r_unit_A;
    
    % Recompute all other unit vectors.
    v_unit_A = orientation_finder(dro_vel_A);
    y_prev_A = y_unit_A;
    y_unit_A = target_finder(dro_pos_A,tar_pos_V);
    y_diff_A = y_prev_A - y_unit_A;
    for i = 1:12
        y_factor_V(i) = 1*norm(y_diff_A(i,:));
    end
    y_repulsion_A = y_unit_A.*y_factor_V;
    y_repulsion_V = reshape(y_repulsion_A',1,24);
    
    % Compute the repulsion terms.
    v_repulsion_V = sum(v_unit_A,1);
    v_sum_V(t,:) = v_repulsion_V;
    
    % Compute the 'angle inference' vector.
    R = r_factor_A;
    R = [R(:,1),R(:,1),R(:,2),R(:,2),R(:,3),R(:,3),R(:,4),R(:,4)...
        ,R(:,5),R(:,5),R(:,6),R(:,6),R(:,7),R(:,7),R(:,8),R(:,8)...
        ,R(:,9),R(:,9),R(:,10),R(:,10),R(:,11),R(:,11),R(:,12),R(:,12)];
    r_factor_A = R;
    gamma_A = r_factor_A.*r_unit_A;
    % This vector is to counteract any closeness between drones.
    gamma_V = sum(gamma_A);
    
    % This form is to counteract any shared trajectories between drones.
    % First find the approximated y unit vectors for all drones, by
    % assuming that they are perpendicular to the motion of travel.
    dash_y_A = ([0, pi/2; -pi/2, 0]*v_unit_A')';
    for i = 1:12
        dash_y_A(i,:) = dash_y_A(i,:)/norm(dash_y_A(i,:));
    end
    
    % Now find the inner products of all of these vectors.
    dash_inner_A = zeros(12);
    for i = 1:11
        for j = (i+1):12
            dash_inner_A(i,j) = dot(dash_y_A(i,:),dash_y_A(j,:));
        end
    end
    dash_inner_A = dash_inner_A + dash_inner_A';
    
    % Choose the value of the worst performing pair, i.e., the inner
    % product which is closest to 1. Note that <y(i),y(j)> = -1 is NOT a
    % problem, as they are facing in opposite directions. We actually want
    % to use (1 - X) where X is the worst value. NOTE: This may not work,
    % so instead we can penalise in terms of each drone's personal worst.
%     dash_factor_S = min(min(1 - dash_inner_A));
    dash_factor_V = min(1 - dash_inner_A);
    dash_factor_V = reshape(repmat(dash_factor_V,2,1),24,1)';
    dash_acos_A = acos(dash_inner_A);
    dash_mod_A = mod(dash_acos_A, 2*pi/12);
    dash_sum_V = sum(dash_mod_A,1)/12;
    dash_sum_V = reshape(repmat(dash_sum_V,2,1),24,1)';
    % Divide by the penalizing factor, which will make everything increase.
%     dash_sum_V = dash_sum_V/dash_factor_S;
    dash_sum_V = dash_sum_V./(dash_factor_V).^2;
%     dash_sum_V = dash_sum_V./(dash_factor_V).^zeta;
    % Find the order of magnitude by which we can affect the 'bad' vectors.
    dash_magnitude_V = floor(floor(log10(dash_sum_V))/6);
        
    % Update the drone trajectories.
    dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt_S;
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
        dro_vel_prev_V - ...
        gamma*gamma_V - ...
        eta*y_repulsion_V + randn(1,24).*dash_magnitude_V*min(50,t/10))*dt_S;
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

end
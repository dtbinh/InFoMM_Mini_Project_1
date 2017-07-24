function [] = four_no_collapse(L,T,a,b,g,m)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the EXTENDED version of the tracking problem, using
%     FOUR individual drones and a single object to be tracked:
%     dV/dt = a*y - b*sum(v(j)) - b*V + g*sum(dr(ij)/dt)*r(rij)
%                 - m*(dy/dt)*y
% INPUT: 
%     L: {float} Length of the simulation (seconds).
%     T:   {int} Number of data points wanted in the simulation.
%     a: {float} 'alpha' in the original system equations.
%     b: {float} 'beta' in the original system equations.
%     g: {float} 'gamma' in the amended system equations.
%     m: {float} 'mu' in the amended system equations.
% OUTPUT:
%     : {}

%% Example
% four_no_collapse(100,5001,10,10,1,0)

%%
keepvars = {'L','T','a','b','g','m'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
Y1 = @(t) 0*cos(t/20);
Y2 = @(t) 0*sin(t/20);
tar_pos_V = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
dro_pos_A = repmat(tar_pos_V,4,1) + 2*randn(4,2);

% Far-distance initialisation. NOTE: Without noise, this can find that the
% circular orbit is unstable, as it eventually breaks.
% dro_pos_A = [-10,10; 10,10; 10,-10; -10,-10] + 0.0*randn(4,2);

% Stationary initialisation.
dro_vel_A = zeros(4,2);  

% Circular orbit initialisation.
y_unit_A = target_finder(dro_pos_A,tar_pos_V);
dro_vel_A = 1*([0, -1; 1 0]*y_unit_A')';

% Compute all unit vectors {r,v,y} defined in the original formulation.
r_unit_A = direction_finder(dro_pos_A);
r_prev_A = r_unit_A;
v_unit_A = orientation_finder(dro_vel_A);
y_unit_A = target_finder(dro_pos_A,tar_pos_V);

% If wanted, plot the initial positions of all individuals, to check that
% there is no initial problem, i.e., no initial overlapping of drones.
if 1
    initial_plot = figure();
    hold on;
    
    % Plot the locations of the drones and the target.
    scatter(dro_pos_A(:,1),dro_pos_A(:,2), 100, 'g');
    quiver(dro_pos_A(:,1),dro_pos_A(:,2),...
           dro_vel_A(:,1),dro_vel_A(:,2), 'b')
    scatter(tar_pos_V(1), tar_pos_V(2), 100, 'rx');
    
    % Plot the direction vectors from drones to all other objects.
    for i = 1:4
        quiver(repmat(dro_pos_A(i,1),4,1),repmat(dro_pos_A(i,2),4,1),...
        -.3*r_unit_A(:,2*i-1),-.3*r_unit_A(:,2*i), 'k');
    end    
    axis equal;
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
gamma = g;
mu = m;
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
h1 = animatedline('Color','r','MaximumNumPoints',30);
h2 = animatedline('Color','b','MaximumNumPoints',30);
h3 = animatedline('Color','g','MaximumNumPoints',30);
h4 = animatedline('Color','m','MaximumNumPoints',30);
h5 = animatedline('Color','k','MaximumNumPoints',30);
[R_calc] = radius_calc(a,b,g,0)
sqax = R_calc + 10;
axis([-sqax,sqax,-sqax,sqax]);
legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
shg;

% Keep track of the repulsion factor.
v_sum_V = zeros(T,2);
repulsion_A = zeros(4);
y_factor_V = zeros(4,1);

for t = 2:T
    
    if mod(t,100) == 0
        % Time counter, to ensure the code is running.
        t_count = t
        [R_est] = radius_est(dro_pos_A,4)
       % pause
    end
    
    % Reshape the arrays.
    dro_pos_A = reshape(dro_pos_prev_V,2,4)';
    dro_vel_A = reshape(dro_vel_prev_V,2,4)';
    tar_pos_V = [Y1(all_time_V(t)),Y2(all_time_V(t))];

    % Recompute all unit vectors.
    r_unit_A = direction_finder(dro_pos_A);
    
    % Compute the factor from which we shall try to avoid close distances,
    % i.e, we compute the differences for all r(i,j) from t-1 to t.
    r_diff_A = r_unit_A - r_prev_A;
    r_factor_A = zeros(4);
    for i = 1:3
        for j = (i+1):4
            r_factor_A(i,j) = ...
            1*norm([r_diff_A(i,2*j-1),r_diff_A(i,2*j)]/dt);
        end
    end
    % This builds the array of ||r(i,t)@(t) - r(i,j)@(t-1)||.
    r_factor_A = r_factor_A + r_factor_A';
    r_prev_A = r_unit_A;
    
    % Recompute all other unit vectors.
    v_unit_A = orientation_finder(dro_vel_A);
    y_prev_A = y_unit_A;
    y_unit_A = target_finder(dro_pos_A,tar_pos_V);
    y_diff_A = y_unit_A - y_prev_A;
    for i = 1:4
        y_factor_V(i) = 1*norm(y_diff_A(i,:))/dt;
    end
    y_repulsion_A = y_unit_A.*y_factor_V;
    y_repulsion_V = reshape(y_repulsion_A',1,8);
    
    % Compute the repulsion terms.
    v_repulsion_V = sum(v_unit_A,1);
    v_sum_V(t,:) = v_repulsion_V;
    
    % Compute the 'angle inference' vector.
    R = r_factor_A;
    R = [R(:,1),R(:,1),R(:,2),R(:,2),R(:,3),R(:,3),R(:,4),R(:,4)];
    r_factor_A = R;
    gamma_A = r_factor_A.*r_unit_A;
    % This vector is to counteract any closeness between drones.
    gamma_V = sum(gamma_A);
    
    % Update the drone trajectories.
    dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
    dro_pos_prev_V = dro_traj_A(t,:);  
    
    % Update the drone velocities.
    dro_direc_A(t,:) = dro_vel_prev_V + ...
        (alpha*reshape(y_unit_A',1,8) - ...
        beta*repmat(v_repulsion_V,1,4) + ...
        beta*reshape(v_unit_A',1,8) - ...
        dro_vel_prev_V - ...
        gamma*gamma_V - ...
        mu*y_repulsion_V)*dt;
    dro_vel_prev_V = dro_direc_A(t,:);
    
    % Update the drone positions.
    addpoints(g1,tar_trajectory_A(t,1),tar_trajectory_A(t,2));
    addpoints(g2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(g3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(g4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(g5,dro_traj_A(t,7),dro_traj_A(t,8));
    
    % Add new points to the trajectory lines.
    addpoints(h1,tar_trajectory_A(t,1),tar_trajectory_A(t,2));
    addpoints(h2,dro_traj_A(t,1),dro_traj_A(t,2));
    addpoints(h3,dro_traj_A(t,3),dro_traj_A(t,4));
    addpoints(h4,dro_traj_A(t,5),dro_traj_A(t,6));
    addpoints(h5,dro_traj_A(t,7),dro_traj_A(t,8));
    
    drawnow;
    
    if t == T-1
        rad_av = (norm(dro_pos_A(1,:)) + norm(dro_pos_A(2,:)) + ...
            norm(dro_pos_A(3,:)) + norm(dro_pos_A(4,:)))/4
        pause
    end
        
end

end
function [] = equi_tracking_with_angle_inference(L,T,a,b,g,e,h)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     four individual drones and a single object to be tracked. In this
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
%     h: {float} 'iota' in the amended system equations.
% OUTPUT:
%     : {}

%% Example
% equi_tracking_with_angle_inference(100,5001,1.5,0.5,10,1,1) {NOT GOOD}
% equi_tracking_with_angle_inference(100,5001,10,10,10,100,1) w/ slow circ
% equi_tracking_with_angle_inference(100,5001,5,5,50,100,1) w/ stationary

%% INCOMPLETE

%%
keepvars = {'L','T','a','b','g','e','h'};
clearvars('-except', keepvars{:}); close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
Y1 = @(t) 10*sin(t/20);
Y2 = @(t) 10*cos(t/20);
% Y1 = @(t) 0.1*cos(t/20);
% Y2 = @(t) 0.1*sin(t/20);
T_pos_vec = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
D_pos_array = repmat(T_pos_vec,4,1) + 5*randn(4,2);

% Fixed pos. initialisation.
% D_pos_array = [-3 -1 1 3; 
%                  -2 -2 -2 -2]'; 

% Stationary initialisation.
D_vel_array = zeros(4,2);  

% Random vel. initialisation.
% D_vel_array = randn(4,2);    

% Compute all unit vectors {r,v,y} defined in the original formulation.
r_unit_array = direction_finder(D_pos_array);
r_previous = r_unit_array;
v_unit_array = orientation_finder(D_vel_array);
y_unit_array = target_finder(D_pos_array,T_pos_vec);

% Initialise ortbiting.
D_vel_array = ([0, -pi/2; pi/2 0]*y_unit_array')'

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
all_time_V = linspace(0,L,T);
T_trajectory = [Y1(all_time_V);Y2(all_time_V)]';

% Control parameters.
alpha = a;
beta = b;
gamma = g;
eta = e;
iota = h;
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
axis([-15,15,-15,15]);
% axis([-30,30,-30,30]);
legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
shg;

v_sum_vec = zeros(T,2);
repulsion_array = zeros(4);
y_factor_vec = zeros(4,1);
y_dot_approx = zeros(4);

for t = 2:T
    
    % Reshape the arrays.
    D_pos_array = reshape(D_pos_previous,2,4)';
    D_vel_array = reshape(D_vel_previous,2,4)';
    T_pos_vec = [Y1(all_time_V(t)),Y2(all_time_V(t))];

    % Recompute all unit vectors.
    r_unit_array = direction_finder(D_pos_array);
    
    % Compute the factor from which we shall try to avoid close distances.
    r_diff_array = r_previous - r_unit_array;
    r_factor_array = zeros(4);
    for i = 1:3
        for j = (i+1):4
            r_factor_array(i,j) = ...
            1*norm([r_diff_array(i,2*j-1),r_diff_array(i,2*j)]);
        end
    end
    r_factor_array = r_factor_array + r_factor_array';
    r_previous = r_unit_array;
    
    % Recompute all other unit vectors.
    v_unit_array = orientation_finder(D_vel_array);
    y_previous = y_unit_array;
    y_unit_array = target_finder(D_pos_array,T_pos_vec);
    y_diff_vec = y_previous - y_unit_array;
    for i = 1:4
        y_factor_vec(i) = 1*norm(y_diff_vec(i,:));
    end
    y_repulsion = y_unit_array.*y_factor_vec;
    y_repulsion = reshape(y_repulsion',1,8);
    
    % Compute the repulsion terms.
    v_repulsion = sum(v_unit_array,1);
    v_sum_vec(t,:) = v_repulsion;
    
    % The above is a bit ad-hoc, but basically we want to force our facing
    % velocities to be in partition of 2pi/N, but we do NOT want the drones
    % to collapse onto each other.
    
    %% Approximating y(j) directions.
    
    % This all needs going through thoroughly, because it doesn't do
    % exactly what I want it to. I think it's because when two drones
    % collapse to one another, their approximated y(j) are very close,
    % however they are BOTH balanced out by the corresponding opposite pair
    % 
    y_approx_array = ([0 pi/2; -pi/2 0]*v_unit_array')';
    for i = 1:4
        y_approx_array(i,:) = y_approx_array(i,:)./norm(y_approx_array(i,:));
    end
    y_dot_approx = zeros(4);
    for i = 1:4
        for j = 1:4
            y_dot_approx(i,j) = dot(y_approx_array(i,:),y_approx_array(j,:));
        end
    end
    y_dot_approx = y_dot_approx - diag(diag(y_dot_approx));
    division_factor = 1 - abs(y_dot_approx);
    acos_factor = acos(y_dot_approx);
    mod_factor = mod(acos_factor,2*pi/4);
    full_factor = mod_factor./division_factor;
    same_direction = sum(full_factor,1)';
    if t < 10
        same_direction = ones(4,1);
    end
    continuation_factor = D_pos_array./same_direction;
    continuation_factor = reshape(continuation_factor',1,8);
    
    
    %%
    
    repulsion_factor = sum(sum(repulsion_array));
    if repulsion_factor < 0.1;
        repulsion_factor = 0.1;
    end
    
    % Compute the 'angle inference' vector.
    R = r_factor_array;
    R = [R(:,1),R(:,1),R(:,2),R(:,2),R(:,3),R(:,3),R(:,4),R(:,4)];
    r_factor_array = R;
    gamma_array = r_factor_array.*r_unit_array;
    % This vector is to counteract any closeness between drones.
    gamma_vec = sum(gamma_array);
    
    % Update the drone trajectories.
    D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
    D_pos_previous = D_traj_array(t,:);  
    
    % Update the drone velocities.
    D_dir_array(t,:) = D_vel_previous + ...
        (alpha*reshape(y_unit_array',1,8) - ...
        beta*repmat(v_repulsion,1,4) + ...
        beta*reshape(v_unit_array',1,8) - ...
        D_vel_previous + ...
        gamma*gamma_vec - ...
        eta*y_repulsion + ...
        iota*continuation_factor)*dt;
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
% norm_sum_vec = zeros(1,T);
% for i = 1:T
%     norm_sum_vec(i) = norm(v_sum_vec(i,:));
% end
% figure();
% plot(norm_sum_vec);
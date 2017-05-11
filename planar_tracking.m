function [] = planar_tracking(N,L,T,a,b)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     four individual drones and a single object to be tracked. In this
%     formulation, we will impose all parameters, as well as the movement
%     of the tracked object. These may become inputs in later iterations of
%     this code. NOTE: Having 'N' as an input variable means that we cannot
%     easily plot the trajectories, so it is prudent to only visualise the
%     trajectories for a specified N < VAL.
% INPUT: 
%     N: {int} Number of drones in the model.
%     L: {float} Length of the simulation (seconds).
%     T: {int} Number of data points wanted in the simulation.
%     a: {float} 'alpha' in the original system equations.
%     b: {float} 'beta' in the original system equations.
% OUTPUT:
%     : {}

%% Examples
% planar_tracking(4,100,5001,1.5,0.5)

%%
keepvars = {'N','L','T','a','b'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
Y1 = @(t) sin(t);
Y2 = @(t) cos(t);
T_pos_vec = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
D_pos_array = repmat(T_pos_vec,N,1) + randn(N,2); 

% Stationary initialisation.
D_vel_array = zeros(N,2);  

% Random vel. initialisation.
% D_vel_array = randn(N,2);     

% Compute all unit vectors {r,v,y} defined in the original formulation.
[r_unit_array] = direction_finder(D_pos_array);
[v_unit_array] = orientation_finder(D_vel_array);
[y_unit_array] = target_finder(D_pos_array,T_pos_vec);

% If wanted, plot the initial positions of all individuals, to check that
% there is no initial problem, i.e., no initial overlapping of drones.
if 1
    % Only plot figures if the system is not too large.
    if N < 10
        figure();
        hold on;
        
        % Plot the locations of the drones and the target.
        scatter(D_pos_array(:,1),D_pos_array(:,2), 100, 'g');
        quiver(D_pos_array(:,1),D_pos_array(:,2),...
               D_vel_array(:,1),D_vel_array(:,2), 'b')
        scatter(T_pos_vec(1), T_pos_vec(2), 100, 'rx');
        
        % Plot the direction vectors from drones to all other objects.
        for i = 1:N
            quiver(repmat(D_pos_array(i,1),N,1),...
                repmat(D_pos_array(i,2),N,1),-.3*r_unit_array(:,2*i-1),...
                -.3*r_unit_array(:,2*i), 'k');
            quiver(D_pos_array(i,1),D_pos_array(i,2),...
                .3*y_unit_array(i,1),.3*y_unit_array(i,2),'r');
        end

        axis equal;
        shg;
    end
end

%%

% Choose a time-length and number of time-steps, to then calculate the
% entire deterministic trajectory of the target.
dt = L/(T-1);
all_time = linspace(0,L,T);
T_trajectory = [Y1(all_time);Y2(all_time)]';

% Control parameters.
alpha = a;
beta = b;
vec_repulsion = sum(v_unit_array,1);

% Define the functional form of dV/dt = F (Eq. 21) with the argument being 
% the drone being calculated.
F = @(i) alpha*y_unit_array(i,:) - beta*vec_repulsion + ...
         beta*v_unit_array(i,:) - D_vel_array(i,:);

%%

if N < 10
    plot_maker(N,T,dt,D_pos_array,D_vel_array,T_pos_vec,...
                 v_unit_array,Y1,Y2,all_time,alpha,beta);
else
    sprintf('No plots will be made. N >= 10')
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
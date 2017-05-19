function [] = tracking_N_drones(N,L,T,a,b)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     N individual drones and a single object to be tracked. In this
%     formulation, we will impose all parameters, as well as the movement
%     of the tracked object. These may become inputs in later iterations of
%     this code. NOTE: Having 'N' as an input variable means that we cannot
%     easily plot the trajectories, so it is prudent to only visualise the
%     trajectories for a specified N < 10.
% INPUT: 
%     N: {int} Number of drones in the model.
%     L: {float} Length of the simulation (seconds).
%     T: {int} Number of data points wanted in the simulation.
%     a: {float} 'alpha' in the original system equations.
%     b: {float} 'beta' in the original system equations.
% OUTPUT:
%     : {}

%% Example
% tracking_N_drones(4,100,5001,1.5,0.5)

%% INCOMPLETE - Reliant on PLOT_MAKER
keepvars = {'N','L','T','a','b'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Set up the position and velocities of the individuals, setting the
% tracked object to start at the origin, or some other point initialised by
% the position [f(0), g(0)], where the position of the tracked object is
% governed by the vector function [f(t), g(t)].

% Functional initialisation of the target's movement.
Y1 = @(t) sin(t/20);
Y2 = @(t) cos(t/20);
T_pos_vec = [Y1(0),Y2(0)];

% Random pos. initialisation AROUND the target
dro_pos_A = repmat(T_pos_vec,N,1) + randn(N,2); 

% Stationary initialisation.
dro_vel_A = zeros(N,2);  

% Random vel. initialisation.
% dro_vel_A = randn(N,2);     

% Compute all unit vectors {r,v,y} defined in the original formulation.
r_unit_A = direction_finder(dro_pos_A);
v_unit_A = orientation_finder(dro_vel_A);
y_unit_A = target_finder(dro_pos_A,T_pos_vec);

% If wanted, plot the initial positions of all individuals, to check that
% there is no initial problem, i.e., no initial overlapping of drones.
if 1
    % Only plot figures if the system is not too large.
    if N < 10
        figure();
        hold on;
        
        % Plot the locations of the drones and the target.
        scatter(dro_pos_A(:,1),dro_pos_A(:,2), 100, 'g');
        quiver(dro_pos_A(:,1),dro_pos_A(:,2),...
               dro_vel_A(:,1),dro_vel_A(:,2), 'b')
        scatter(T_pos_vec(1), T_pos_vec(2), 100, 'rx');
        
        % Plot the direction vectors from drones to all other objects.
        for i = 1:N
            quiver(repmat(dro_pos_A(i,1),N,1),...
                repmat(dro_pos_A(i,2),N,1),-.3*r_unit_A(:,2*i-1),...
                -.3*r_unit_A(:,2*i), 'k');
            quiver(dro_pos_A(i,1),dro_pos_A(i,2),...
                .3*y_unit_A(i,1),.3*y_unit_A(i,2),'r');
        end

        axis square;
        shg;
    end
end

% Choose a time-length and number of time-steps, to then calculate the
% entire deterministic trajectory of the target.
dt = L/(T-1);
all_time_V = linspace(0,L,T);
tar_traj_A = [Y1(all_time_V);Y2(all_time_V)]';

% Control parameters.
alpha = a;
beta = b;
vec_repulsion_V = sum(v_unit_A,1);

%%

if N < 10
    plot_maker(N,T,dt,dro_pos_A,dro_vel_A,T_pos_vec,...
                 v_unit_A,Y1,Y2,all_time_V,alpha,beta);
else
    sprintf('No plots will be made. N >= 10')
end

end
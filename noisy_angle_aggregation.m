function [] = noisy_angle_aggregation(N,r,a,b,del,T,mu)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Implementation of the simple swarm aggregation model given by (3.7)
%     in 'Swarm Stability anf Optmization', only now we have a noise
%     parameter 'mu' which controls the error in estimating the angle
%     from one drone to another. It is likely that the effect of 'mu' will
%     be coupled with the choices for attraction and repulsion.
% INPUT: 
%     N:   {int} Number of drones.
%     r: {float} 'Radius' of the drone.
%     a: {float} Attraction factor.
%     b: {float} Repulsion factor.
%   del: {float} Equilibrium distance to test.
%     T:   {int} Number of timesteps.
%    mu: {float} Noise parameter.
% OUTPUT:
%      : {}

%% Example
% [] = noisy_angle_aggregation(30,1,0.001,1,3,5000,0.1)

%%
keepvars = {'N','r','a','b','del','T','mu'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Initial positions, with check for overlapping.
crash_flag = 1;
while crash_flag ~= 0
    pos_vec = 2*del*randn(N,2);
    [idx_array,d] = knnsearch(pos_vec, pos_vec, 'k', 2);
    crash_flag = sum(d(:,2) < 2*r);
end

% Compute the centroid of all drones.
centroid = sum(pos_vec,1)/N;

% Plot positions and the centroid.
figure();
hold on;
scatter(pos_vec(:,1),pos_vec(:,2),100,'b');
scatter(centroid(1),centroid(2),100,'r');
% ref_frame = [floor(min(pos_vec(:,1)))-1 ceil(max(pos_vec(:,1)))+1 ...
%       floor(min(pos_vec(:,2)))-1 ceil(max(pos_vec(:,2)))+1];
ref_frame = [-5*del 5*del -5*del 5*del];
axis(ref_frame);
shg;

% Find the coefficient c to build the function g(y).
c = del^2/log(b/a);

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
func = @(y) -y*(a - b*exp(-1*(norm(y)^2)/c));

% Updating scheme for drone position.
new_pos_vec = zeros(size(pos_vec));

% Plot the trajectories of the drones.
for t = 1:T
    if mod(t,50) == 0;
        t
    end
    new_dir_vec = zeros(size(pos_vec));
    for i = 1:N
        for j = 1:N
            % Introduce noise as a Gaussian error in all directions.
            new_dir_vec(i,:) = new_dir_vec(i,:) + ...
                func(pos_vec(i,:) - pos_vec(j,:) + mu*randn(1,2));
        end
    end
    new_pos_vec = pos_vec + new_dir_vec;
    pos_vec = new_pos_vec;
    centroid = sum(pos_vec,1)/N;
    
    clf;
    hold on;
    scatter(pos_vec(:,1),pos_vec(:,2),100,'b');
    scatter(centroid(1),centroid(2),100,'r');
    axis(ref_frame);
    drawnow;
end


    
    

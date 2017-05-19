function [] = finite_size_aggregation(N,r,a,b,c,T)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Implementation of the simple swarm aggregation model given by (3.7)
%     in 'Swarm Stability anf Optmization'.
% INPUT: 
%     N:   {int} Number of drones.
%     r: {float} 'Radius' of the drone.
%     a: {float} Attraction factor.
%     b: {float} Repulsion factor.
%     c: {float} Attraction-Repulsion factor. Note now that we do not
%        specifiy 'del' anymore, as it will be computed from {a,b,c}.
%        Instead, we hard-limit the radial size of the drones so that they 
%        do not overlap.
%     T:   {int} Number of timesteps.
% OUTPUT:
%      : {}

%% Example
% [] = finite_size_aggregation(10,1,0.001,1,1,5000)

%%
keepvars = {'N','r','a','b','c','T'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Initial positions, with check for overlapping.
crash_flag = 1;
while crash_flag ~= 0
    pos_vec = 5*r*randn(N,2);
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
ref_frame = [floor(min(pos_vec(:,1)))-1 ceil(max(pos_vec(:,1)))+1 ...
      floor(min(pos_vec(:,2)))-1 ceil(max(pos_vec(:,2)))+1];
axis(ref_frame);
shg;

% Find the equilibrium distance.
delta = sqrt(c*log(b/a))

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
func = @(y) -y*(a - b/(norm(y)^2 - 4*r^2)^2);

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
            new_dir_vec(i,:) = new_dir_vec(i,:) + ...
                func(pos_vec(i,:) - pos_vec(j,:));
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


    
    

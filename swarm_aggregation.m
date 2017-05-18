function [] = swarm_aggregation(N,r,a,b,del,T)

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
%   del: {float} Equilibrium distance to test.
%     T:   {int} Number of timesteps.
% OUTPUT:
%      : {}

%% Example
% [] = swarm_aggregation(30,1,0.001,1,5,5000)

%%
keepvars = {'N','r','a','b','del','T'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Initial positions, with check for overlapping.
crash_flag = 1;
while crash_flag ~= 0
    pos_V = 10*randn(N,2);
    % Check that no neighbours are within a radius of 2r.
    [~,d] = knnsearch(pos_V, pos_V, 'k', 2);
    crash_flag = sum(d(:,2) < 2*r);
end

% Compute the centroid of all drones.
centroid_V = sum(pos_V,1)/N;

% Plot positions and the centroid.
figure();
hold on;
scatter(pos_V(:,1),pos_V(:,2),100,'b');
scatter(centroid_V(1),centroid_V(2),100,'r');
% Set a reference frame for the axes, assuming that all drones will
% converge to WITHIN.
ref_frame = [floor(min(pos_V(:,1)))-1 ceil(max(pos_V(:,1)))+1 ...
      floor(min(pos_V(:,2)))-1 ceil(max(pos_V(:,2)))+1];
axis([min(ref_frame), max(ref_frame), min(ref_frame), max(ref_frame)]);
axis square
shg;

% Find the coefficient c to build the function g(y).
c = del^2/log(b/a);

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
att_rep_F = @(y) -y*(a - b*exp(-1*(norm(y)^2)/c));

% Updating scheme for drone position.
new_pos_V = zeros(size(pos_V));

% Plot the trajectories of the drones.
for t = 1:T
    % Time counter, to ensure the code is running.
    if mod(t,50) == 0;
        t_count = t
    end
    
    % Update the orientation.
    new_dir_vec = zeros(size(pos_V));
    for i = 1:N
        for j = 1:N
            new_dir_vec(i,:) = new_dir_vec(i,:) + ...
                att_rep_F(pos_V(i,:) - pos_V(j,:));
        end
    end
    % This can change to be '+ dt*new_dir_V' if needed.
    new_pos_V = pos_V + new_dir_vec;
    pos_V = new_pos_V;
    centroid_V = sum(pos_V,1)/N;
    
    % Is there a way to update the figure with scatter plots more easily
    % than clearing the figure and then completely re-drawing it?
    clf;
    hold on;
    scatter(pos_V(:,1),pos_V(:,2),100,'b');
    scatter(centroid_V(1),centroid_V(2),100,'r');
    axis(ref_frame);
    drawnow;
end

end

end
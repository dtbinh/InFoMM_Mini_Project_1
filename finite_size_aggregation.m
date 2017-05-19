function [] = finite_size_aggregation(N,r,a,b,c,T)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Implementation of the simple swarm aggregation model given by (3.7)
%     in 'Swarm Stability anf Optmization', using a finite size limitation.
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
% [] = finite_size_aggregation(20,1,0.001,1,0.01,5000)

%%
keepvars = {'N','r','a','b','c','T'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Initial positions, with check for overlapping.
crash_flag = 1;
while crash_flag ~= 0
    pos_V = 0.5*N*r^2*randn(N,2);
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
axis square;
shg;

% Compute the equilibrium distance.
delta = sqrt(c*log(b/a));

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
att_rep_F = @(y) -y*(a - b/(norm(y)^2 - 4*r^2)^2);

% Updating scheme for drone position.
new_pos_V = zeros(size(pos_V));

% Plot the trajectories of the drones.
for t = 1:T
    % Time counter, to ensure the code is running.
    if mod(t,50) == 0
        t_count = t
    end
    
    % Update the orientation.
    new_dir_V = zeros(size(pos_V));
    for i = 1:N
        for j = 1:N
            new_dir_V(i,:) = new_dir_V(i,:) + ...
                att_rep_F(pos_V(i,:) - pos_V(j,:));
        end
    end
    % This can change to be '+ dt*new_dir_V' if needed.
    new_pos_V = pos_V + new_dir_V;
    pos_V = new_pos_V;
    centroid_V = sum(pos_V,1)/N;
    
    % Is there a way to update the figure with scatter plots more easily
    % than clearing the figure and then completely re-drawing it?
    clf;
    hold on;
    scatter(pos_V(:,1),pos_V(:,2),100,'b');
    scatter(centroid_V(1),centroid_V(2),100,'r');
    axis([min(ref_frame), max(ref_frame), min(ref_frame), max(ref_frame)]);
    axis square
    drawnow;
    
end

end
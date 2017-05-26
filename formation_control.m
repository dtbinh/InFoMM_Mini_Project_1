function [] = formation_control(N,r,A,B,DEL,T)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Implementation of the formation control model given by (Eq?)
%     in 'Swarm Stability and Optmization'.
% INPUT: 
%     N:   {int} Number of drones.
%     r: {float} 'Radius' of the drone.
%     A:   {mat} Attraction factors.
%     B:   {mat} Repulsion factors
%   DEL:   {mat} Equilibrium distances to test.
%     T:   {int} Number of timesteps.
% OUTPUT:
%      : {}

%% Examples
% F = 3*[0 1 1 sqrt(2); 1 0 sqrt(2) 1; 1 sqrt(2) 0 1; sqrt(2) 1 1 0];
% formation_control(4,1,0.01*ones(4),5*ones(4),F,5000)
% G = 5*[0 1 1 2 sqrt(3) 2; 1 0 1 1 1 sqrt(3); 1 1 0 sqrt(3) 1 1;
%        2 1 sqrt(3) 0 1 2; sqrt(3) 1 1 1 0 1; 2 sqrt(3) 1 2 1 0]
% formation_control(6,1,0.04*ones(6),1*ones(6),G,5000)
% formation_control(N,1,0.04*ones(N),1*ones(N),D,5000)


%%
keepvars = {'N','r','A','B','DEL','T'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Initial positions, with check for overlapping.
crash_flag = 1;
while crash_flag ~= 0
    pos_V = 30*randn(N,2);
    % Check that no neighbours are within a radius of 2r.
    [~,d] = knnsearch(pos_V, pos_V, 'k', 2);
    crash_flag = sum(d(:,2) < 2*r);
end

% Compute the centroid of all drones.
centroid_V = sum(pos_V,1)/N;

% Plot positions and the centroid.
fig1 = figure();
hold on;
scatter(pos_V(:,1),pos_V(:,2),100,'b');
scatter(centroid_V(1),centroid_V(2),100,'r');
% Set a reference frame for the axes, assuming that all drones will
% converge to WITHIN.
ref_frame = [floor(min(pos_V(:,1)))-1 ceil(max(pos_V(:,1)))+1 ...
      floor(min(pos_V(:,2)))-1 ceil(max(pos_V(:,2)))+1];
axis(ref_frame);
axis([min(ref_frame), max(ref_frame), min(ref_frame), max(ref_frame)]);
axis square;
shg;

% Find the coefficient c(i,j) to build the Attraction-Repulsion function.
C = zeros(N);
for i = 1:N
    for j = 1:N
        C(i,j) = DEL(i,j)^2/log(B(i,j)/A(i,j));
    end
end

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
att_rep_F = @(y,i,j) -y*(A(i,j) - B(i,j)*exp(-1*(norm(y)^2)/C(i,j)));

% Updating scheme for drone position.
new_pos_V = zeros(size(pos_V));
new_norm_diff = 0;
counter = 1;

% Plot the trajectories of the drones.
for t = 1:T
    if mod(t,50) == 0
        % Time counter, to ensure the code is running.
        t_count = t
        %
        dist_A = zeros(N);
        for i = 1:(N-1)
            for j = i+1:N
                dist_A(i,j) = norm(new_pos_V(i) - new_pos_V(j));
            end
        end
        dist_A = dist_A + dist_A';
        diff_A = abs(DEL - dist_A);
        norm(diff_A);
        %
        if abs(new_norm_diff - norm(diff_A)) < 0.1
            % This 'shaking' of the system to find the final equilibrium
            % should be done in a way such that we only perturb the drones
            % who are in the wrong place. This should probably be done by
            % checking which rows have the largest total difference, as
            % drones which are in the right place will have very small
            % differences alongside drones who are ALSO in the right place.
            pos_V = pos_V + counter*randn(N,2);
            counter = counter + 1;
        end
        new_norm_diff = norm(diff_A);
        
    end
    
    % Update the orientation.
    new_dir_V = zeros(size(pos_V));
    for i = 1:N
        for j = 1:N
            if j ~= i
                % NOTE: Added a factor of 0.2 before func(...) to act as an
                % artifical timestep, without changing the balance of the
                % attraction/repulsion. It actually seems to be highly
                % dependent on the size of the system too.
            new_dir_V(i,:) = new_dir_V(i,:) + ...
                0.2*att_rep_F(pos_V(i,:) - pos_V(j,:),i,j);
            else
            end
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
    axis([-20, 20, -20, 20])
    axis square;
    drawnow;
end

end
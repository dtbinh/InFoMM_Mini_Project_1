function [] = formation_tracking(N,r,a,b,del,T)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Implementation of the formation control and tracking model given by 
%     (Eq?) in 'Swarm Stability anf Optmization'.
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
% [] = formation_tracking(30,1,0.001,1,20,5000)

%% INCOMPLETE

%%
keepvars = {'N','r','a','b','del','T'};
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
ref_frame = [floor(min(pos_vec(:,1)))-1 ceil(max(pos_vec(:,1)))+1 ...
      floor(min(pos_vec(:,2)))-1 ceil(max(pos_vec(:,2)))+1];
ref_frame = [-5*del 5*del -5*del 5*del]
axis(ref_frame);
shg;

% Find the coefficient c to build the function g(y).
c = del^2/log(b/a);

% Motion of the target.
f1 = @(t) 0.25 + 0.3*sin(t);
f2 = @(t) 0.9*sin(0.25*t);
target_traj = [f1(0),f2(0)];

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
func = @(y) 0.6*(y - target_traj)*norm(y - target_traj)^2 - ...
            0.4*y*(a - b*exp(-1*(norm(y)^2)/c));

% Updating scheme for drone position.
new_pos_vec = zeros(size(pos_vec));

% Plot the trajectories of the drones.
for t = 1:T
    target_traj = 0.1*[f1(t),f2(t)];
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


    
    

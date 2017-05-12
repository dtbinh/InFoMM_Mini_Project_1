function [] = formation_control(N,r,A,B,DEL,T)

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
%     A:   {mat} Attraction factors.
%     B:   {mat} Repulsion factors
%   DEL:   {mat} Equilibrium distances to test.
%     T:   {int} Number of timesteps.
% OUTPUT:
%      : {}

%% Example
% [] = formation_control(4,1,0.01*ones(4),5*ones(4),1*[0 1 1 sqrt(2);
%                                                      1 0 sqrt(2) 1;
%                                                      1 sqrt(2) 0 1;
%                                                      sqrt(2) 1 1 0],5000)

%%
keepvars = {'N','r','A','B','DEL','T'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Initial positions, with check for overlapping.
crash_flag = 1;
while crash_flag ~= 0
    pos_vec = 10*randn(N,2);
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
axis([min(ref_frame), max(ref_frame), min(ref_frame), max(ref_frame)])
axis square
shg;

% Find the coefficient c to build the function g(y).
C = zeros(N);
for i = 1:N
    for j = 1:N
        C(i,j) = DEL(i,j)^2/log(B(i,j)/A(i,j));
    end
end

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
func = @(y,i,j) -y*(A(i,j) - B(i,j)*exp(-1*(norm(y)^2)/C(i,j)));

% Updating scheme for drone position.
new_pos_vec = zeros(size(pos_vec));

% Plot the trajectories of the drones.
for t = 1:T
    if mod(t,50) == 0;
        t
        dist_array = zeros(N);
        for i = 1:(N-1)
            for j = i+1:N
                dist_array(i,j) = norm(pos_vec(i) - pos_vec(j));
            end
        end
        dist_array = dist_array + dist_array';
        diff = abs(DEL - dist_array)
        norm(diff)
    end
    new_dir_vec = zeros(size(pos_vec));
    for i = 1:N
        for j = 1:N
            if j ~= i
            new_dir_vec(i,:) = new_dir_vec(i,:) + ...
                func(pos_vec(i,:) - pos_vec(j,:),i,j);
            else
            end
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
    axis([min(ref_frame), max(ref_frame), min(ref_frame), max(ref_frame)])
    axis square
    drawnow;
end


    
    

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
% noisy_angle_aggregation(30,1,0.001,1,3,5000,0.1)

%%
keepvars = {'N','r','a','b','del','T','mu'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Initial positions, with check for overlapping.
crash_flag = 1;
while crash_flag ~= 0
    pos_V = 5*del*randn(N,2);
    % Check that no neighbours are within a radius of 2r.
    [~,d] = knnsearch(pos_V, pos_V, 'k', 2);
    crash_flag = sum(d(:,2) < 2*r);
end

% Compute the centroid of all drones.
centroid_V = sum(pos_V,1)/N;
ref_centroid_V = sum(pos_V,1)/N;

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

% Find the coefficient c to build the Attraction-Repulsion function.
c = del^2/log(b/a);

% Attraction-Repulsion function (3.7) in 'Swarm Systems...'
att_rep_F = @(y) -y*(a - b*exp(-1*(norm(y)^2)/c));

% Updating scheme for drone position.
new_pos_V = zeros(size(pos_V));
centroid_diff_V = zeros(T+1,1);
centroid_prev_A = zeros(T+1,2);
centroid_prev_A(1,:) = centroid_V;

% Plot the trajectories of the drones.
for t = 1:T
    if mod(t,50) == 0
        % Time counter, to ensure the code is running.
        t_count = t
    end
    
    % Update the orientation.
    new_dir_vec = zeros(size(pos_V));
    for i = 1:N
        for j = 1:N
            % Introduce noise as a Gaussian error in all directions.
            new_dir_vec(i,:) = new_dir_vec(i,:) + ...
                att_rep_F(pos_V(i,:) - pos_V(j,:) + mu*randn(1,2));
            % It may be preferential to use a different error type here.
        end
    end
    % This can change to be '+ dt*new_dir_V' if needed.
    new_pos_V = pos_V + new_dir_vec;
    pos_V = new_pos_V;
    centroid_V = sum(pos_V,1)/N;
    centroid_diff_V(t+1) = norm(centroid_V - ref_centroid_V);
    centroid_prev_A(t+1,:) = centroid_V;
    
    % Is there a way to update the figure with scatter plots more easily
    % than clearing the figure and then completely re-drawing it?
    clf;
    hold on;
    scatter(pos_V(:,1),pos_V(:,2),100,'b');
    scatter(centroid_V(1),centroid_V(2),100,'r');
    axis(ref_frame);
    drawnow;
end

% Plot centroid error over time
subplot(1,2,1);
plot(centroid_diff_V);
xlabel('Time')
ylabel('Error: ||C_s - C_m||')
shg;

% Plot centroid drift path over time.
subplot(1,2,2);
hold on;
plot(centroid_prev_A(:,1),centroid_prev_A(:,2),'r');
scatter(centroid_prev_A(1,1),centroid_prev_A(1,2),100,'bo')
scatter(centroid_prev_A(end,1),centroid_prev_A(end,2),100,'kx')
legend('Path','Start','End')

end
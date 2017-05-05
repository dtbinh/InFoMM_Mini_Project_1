function [] = drone_calibration(M,dt)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Takes the initial drone position array as input, for N drones, as
%     well as (N-1) velocity vectors, to allow each drone to find the exact
%     position of ALL other drones.
% INPUT: 
%     M:   {array} Initial drone positions.
%     dt: {scalar} Size of timestep.
% OUTPUT:
%     : {}

%% Examples
% [knowledge] = drone_calibration(drone_pos_array)

%%
dt = 0.2;
[N,~] = size(M);
drone_vel_calib = randn(N,2*N);
calc_pos_array = zeros(N,2*N);

for i = 1:N
    drone_vel_calib(i,2*i-1:2*i) = 0;
end

positions = M;
new_positions = reshape(M',1,8) + dt*drone_vel_calib(1,:);
new_positions = reshape(new_positions,2,4)';

directions_orig = direction_finder(M);
directions_new = direction_finder(new_positions);

degree_array = zeros(4,4);
for i = 1:4

%% Plot all facing unit directions
if 1
    figure();
    hold on;
    scatter(positions(:,1),positions(:,2), 100, 'g');
    scatter(0, 0, 100, 'rx');
    shg;
    for i = 1:4
        quiver(repmat(positions(i,1),4,1),repmat(positions(i,2),4,1),...
        -.3*directions_orig(:,2*i-1),-.3*directions_orig(:,2*i), 'k');
    end  
    figure();
    hold on;
    scatter(new_positions(:,1),new_positions(:,2), 100, 'g');
    scatter(0, 0, 100, 'rx');
    shg;
    for i = 1:4
        quiver(repmat(new_positions(i,1),4,1),repmat(new_positions(i,2),4,1),...
        -.3*directions_new(:,2*i-1),-.3*directions_new(:,2*i), 'k');
    end  
end

end
function [A] = orientation_finder(M)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This takes the array of drone velocities and computes the unit
%     orientation for each.
% INPUT: 
%     M: {array} Ordered drone velocities.
% OUTPUT:
%     A: {array} Ordered drone orientations (normed velocities).

%% Examples
% [v_unit_array] = orientation_finder(D_vel_array)

%%

% Initialise the holding matrix - this should work for any velocity vector
% of size N x 2.
[N,~] = size(M);
V = zeros(N,2);

for m = 1:N
    % Check if the drone has collapsed onto the target - if it has, this is
    % a problem, but we will deal with this in a different code...
    v = M(m,:);
    if norm(v) ~= 0
        v = v/norm(v);
    end
    V(m,:) = v;
end

% Output the realised unit vector array.
A = V;
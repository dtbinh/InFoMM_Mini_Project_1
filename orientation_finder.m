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
% [v_unit_orientation_array] = orientation_finder(drone_vel_array)

%%
V = zeros(4,2);
for m = 1:4
    v = M(m,:);
    if norm(v) ~= 0
        v = v/norm(v);
    end
    V(m,:) = v;
end
A = V;
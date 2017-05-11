function [A] = target_finder(M,P)
    
% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This takes the array of drone positions, and the position of the
%     target, to compute the unit direction vector for each drone when
%     facing the target.
% INPUT: 
%     M:  {array} Ordered drone positions.
%     P: {vector} Target position.
% OUTPUT:
%     A:  {array} Ordered unit directions from drone(i) to the target.

%% Example
% [y_unit_array] = target_finder(D_pos_array, T_pos_vec)

%%

% Initialise the holding matrix - this should work for any velocity vector
% of size N x 2.
[N,~] = size(M);
Y = zeros(N,2);

for m = 1:N
    % Find the normalised direction vectors for each drone to the target.
    y = P - M(m,:);
    y = y/norm(y);
    Y(m,:) = y;
end

% Output the realised unit vector array.
A = Y;
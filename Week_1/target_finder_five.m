function [A] = target_finder_five(M,P)
    
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

%% Examples
% [y_unit_target_dir_array] = target_finder(drone_pos_array,target_pos_vec)

%%
Y = zeros(5,2);
for m = 1:5
    y = P - M(m,:);
    y = y/norm(y);
    Y(m,:) = y;
end
A = Y;
function [R] = direction_finder_five(M)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This takes the array of drone positions and computes the unit
%     relative direction between each. We only compute the positive (i<j)
%     directions, to use the negative version when i>j. Note that this code
%     is built specifically to work with the N = 4 system.
% INPUT: 
%     M: {array} Ordered drone positions.
% OUTPUT:
%     R: {array} Unit vectors from drone(i) to drone(j) for i<j.

%% Examples
% [r_unit_direction_array] = direction_finder(drone_pos_array)

%%
A = zeros(4,10);
for m = 1:4
    for n = (m + 1):5
        r = M(m,:) - M(n,:);
        if norm(r) ~= 0
            r = r/norm(r);
        end
        A(m,1 + 2*(n-1)) = r(1);
        A(m,2*n) = r(2);
    end
end
R = A;
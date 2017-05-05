function [R] = direction_finder(M)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This takes the array of drone positions and computes the unit
%     relative direction between each. Note that r(i,j) = -r(j,i).
% INPUT: 
%     M: {array} Ordered drone positions.
% OUTPUT:
%     R: {array} Unit vectors from drone(i) to drone(j).

%% Examples
% [r_unit_direction_array] = direction_finder(drone_pos_array)

%%
A = zeros(4,8);
for m = 1:3
    for n = (m + 1):4
        r = M(n,:) - M(m,:);
        if norm(r) ~= 0
            r = r/norm(r);
        end
        A(m,1 + 2*(n-1)) = r(1);
        A(m,2*n) = r(2);
        A(n,2*m-1) = -r(1);
        A(n,2*m) = -r(2);
    end
end
R = A;
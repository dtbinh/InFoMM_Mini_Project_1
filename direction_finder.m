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
% [r_unit_array] = direction_finder(D_pos_array)

%%

% Initialise the holding matrix - this should work for any position vector
% of size N x 2.
[N,~] = size(M);
A = zeros(N,2*N);

for m = 1:(N-1)
    for n = (m + 1):N
        
        % Distance is an unknown in the system, so we find the normalised
        % unit vectors.
        r = M(n,:) - M(m,:);
        if norm(r) ~= 0
            r = r/norm(r);
        end
        
        % The vectors are entered as V = [v(1),v(2)] into the array, such
        % that A is of the form [V,V,...,V,V; V,V,...,V,V; ...]..
        A(m,1 + 2*(n-1)) = r(1);
        A(m,2*n) = r(2);
        A(n,2*m-1) = -r(1);
        A(n,2*m) = -r(2);
    end
end

% Output the realised unit vector array.
R = A;
function [D,N] = formation_matrix_builder(V)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This code builds the distance matrix for any formation M.
% INPUT: 
%     V:   {vec} Cartesian coordinates, ordered from drone 1,...,N.  
% OUTPUT:
%     D: {array} Matrix of distances between each point, which is then used
%     in the formation algorithm as the exact positions are not needed.
%     N: {int} Number of drones in the system.

%% Example
% [D,N] = formation_matrix_builder([1,1;2,1;3,1;2,2;2,3;1,4;2,4;3,4])

%%
[N,~] = size(V);
D = zeros(N);

%
for i = 1:(N-1)
    for j = (i+1):N
        D(i,j) = norm(V(i,:) - V(j,:));
    end
end

%
D = D + D';

end

%%
% ALI =
% [1,1;4,1;6,1;7,1;8,1;9,1;11,1;12,1;13,1;1,2;4,2;6,2;12,2;1,3;2,3;3,3;
%  4,3;6,3;12,3;1,4;4,4;6,4;12,4;1,5;2,5;3,5;4,5;6,5;11,5;12,5;13,5];

% DAN = [1,1;2,1;3,1;6,1;9,1;11,1;14,1;15,1;1,2;4,2;6,2;9,2;11,2;14,2;15,2;
%  1,3;4,3;6,3;7,3;8,3;9,3;11,3;13,3;15,3;1,4;4,4;6,4;9,4;11,4;12,4;13,4;
%  15,4;1,5;2,5;3,5;6,5;7,5;8,5;9,5;11,5;12,5;15,5];

% HATCH = 
% [2,1;4,1;1,2;3,2;5,2;2,3;4,3;1,4;3,4;5,4;2,5;4,5;1,6;3,6;5,6;2,7;4,7];
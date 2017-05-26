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

% FUCKYOU = 
% [1,1;6,1;7,1;8,1;9,1;11,1;12,1;13,1;14,1;16,1;19,1;22,1;26,1;27,1;28,1;
% 29,1;31,1;32,1;33,1;34,1;1,2;6,2;9,2;11,2;16,2;18,2;22,2;26,2;29,2;31,2;
% 34,2;1,3;2,3;3,3;6,3;9,3;11,3;16,3;17,3;22,3;26,3;29,3;31,3;34,3;1,4;6,4;
% 9,4;11,4;16,4;18,4;21,4;23,4;26,4;29,4;31,4;34,4;1,5;2,5;3,5;4,5;6,5;9,5;
% 11,5;12,5;13,5;14,5;16,5;19,5;20,5;24,5;26,5;27,5;28,5;29,5;31,5;34,5]
function [G] = gamma_summation(N)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This function computes the summation of r_{ij} vectors projected onto
%     the y_{i} centripetal vector.
% INPUT: 
%     N:   {int} Number of drones in the system.
% OUTPUT:
%     G: {float} Total projected sum.

%% Example
% [G] = gamma_summation(4);

%%

if mod(N,2) == 0
    num = N/2;
else
    num = (N-1)/2;
end

tot_val = 0;
for j = 1:num
    tot_val = tot_val + cos((pi - (j/N)*2*pi)/2);
end

G = 2*tot_val;

if mod(N,2) == 0
    G = G - 1;
end

end
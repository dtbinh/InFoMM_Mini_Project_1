function [R] = radius_est(P,N)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This function computes the average radius of the extended
%     system.
% INPUT: 
%     P: {array} Array of positions.
%     N:   {int} Number of drones in the system.
% OUTPUT:
%     R: {float} Expected 'final configuration' radius.

%% Example
% [R] = radius_est(dro_pos_A,12)

%%

R = 0;

for i = 1:N
    R = R + norm(P(i,:));
end

R = R/N;

end
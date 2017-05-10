function [R] = relative_bearing(M)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is the initial (simple) version of the tracking problem, using
%     four individual drones and a single object to be tracked. In this
%     formulation, we will impose all parameters, as well as the movement
%     of the tracked object. These may become inputs in later iterations of
%     this code.
% INPUT: 
%     M: {array} Unit direction vectors for paired drones (i,j).
% OUTPUT:
%     R: {array} Direction for paired drones (i,j).

%% Examples
% [r_angle_array] = relative_bearing(r_unit_direction_array)

%%

[m,~] = size(M);
R = zeros(m);
for m = 1:3
    for n = (m+1):4
        R(m,n) = atan(M(m,2*n)/M(m,1 + 2*(n-1))) + pi/2;
        R(n,m) = R(m,n) - pi;
        if M(m,2*n) > 0 && M(m,1 + 2*(n-1)) > 0
            R(m,n) = R(m,n) + pi;
        end
    end
end

% Transform to degrees if necessary
R = R/(2*pi) * 360;
R(R<0) = R(R<0) + 360;
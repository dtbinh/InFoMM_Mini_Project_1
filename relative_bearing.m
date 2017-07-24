function [R,R_deg] = relative_bearing(M)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This code finds the relative bearing of drone(j) from drone(i), using
%     the unit direction vectors of each.
% INPUT: 
%     M: {array} Unit direction vectors for paired drones (i,j).
% OUTPUT:
%     R: {array} Bearing for paired drones (i,j), in radians.
% R_deg: {array} Bearing for paired drones (i,j), in degrees.

%% Example
% [r_angle_array] = relative_bearing(r_unit_direction_array)

%%

[m,~] = size(M);
R = zeros(m);
R_deg = R;

%
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
R_deg = R/(2*pi) * 360;
R_deg(R_deg<0) = R_deg(R_deg<0) + 360;

end
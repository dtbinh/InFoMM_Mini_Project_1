function [L] = angular_momentum(S,P,Y,N)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This function computes the angular momentum of a system of drones.
% INPUT:
%     S: {array} Array of velocities.
%     P: {array} Array of positions.
%     Y: {array} Unit target directions.
%     N:   {int} Number of drones in the system.
% OUTPUT:
%     R: {float} Expected 'final configuration' radius.

%% Example
% [L] = angular_momentum(dro_vel_A,dro_pos_A,y_unit_A,12)

%%

L_vals_V = zeros(1,N);

% NOTE: Make sure that the drones are travelling clockwise before running.
v_tilde_A = ([0, -1; 1 0]*Y')';

for i = 1:N
    v_tilde_V = v_tilde_A(i,:);
    v_orthog_V = v_tilde_V*(S(i,:)');
    L_vals_V(i) = norm(P(i,:))*v_orthog_V;
end

L = sum(L_vals_V);

end
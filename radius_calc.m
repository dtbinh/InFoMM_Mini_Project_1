function [R] = radius_calc(a,b,g,N)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This function computes the expected radius of the EXTENDED system.
% INPUT: 
%     a: {float} 'alpha' in the model.
%     b: {float} 'beta' in the model.
%     g: {float} 'gamma' in the model.
%     N:   {int} Number of drones in the system.
% OUTPUT:
%     R: {float} Expected 'final configuration' radius.

%% Example
% [R] = radius_calc(10,10,0.1,12)

%%
if N == 0
    G = 1;
else
    G = gamma_summation(N);
end

R = (b^2 + g*b*G)/a;

end
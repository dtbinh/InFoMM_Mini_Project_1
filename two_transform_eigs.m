% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Finds eigenvalues for the N = 2 (discrete) system for different 
%     values of alpha and beta, plotting them on a single viewer.
% INPUT: 
%     : {}
% OUTPUT:
%     : {}

%%
close all;
clear all;
clc;

figure();
hold on;
for a = 1:1:10
    for b = 1:1:10

        M = [0 0 0 0 -b 0 0;
             0 0 0 0 0 -b 0;
             0 0 -1 0 a 0 0;
             0 0 0 -1 0 a 0;
             a^2/b^2 0 -2*a/b^2 0 0 0 1;
             0 a^2/b^3 0 -2*a/b^2 0 0 -1;
             0 0 a/b^2 -a/b^2 0 0 -2];

        E = eig(M);
 
        scatter(real(E),imag(E),'.k');
        
    end
end
xlabel('Real','interpreter','latex');
ylabel('Imag','interpreter','latex');
shg;
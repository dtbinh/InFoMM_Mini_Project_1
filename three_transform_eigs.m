% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Finds eigenvalues for the N = 3 (discrete) system for different 
%     values of alpha and beta, plotting them on a single viewer.
% INPUT: 
%     : {}
% OUTPUT:
%     : {}

%%
close all;
clear all;
clc;

TH12 = 2*pi/3;
TH13 = 4*pi/3;
TH23 = 2*pi/3;

figure();
hold on;
for a = 1:1:50
    for b = 1:1:50

        M = [0           0          0            0                                       0                                       0                                      -b  0  0  b                          0                          0;
             0           0          0            0                                       0                                       0                                       0 -b  0  0                          b                          0;
             0           0          0            0                                       0                                       0                                       0  0 -b  0                          0                          b;
             0           0          0           -1                                       0                                       0                                       a  0  0 -a+b*sin(TH12)+b*sin(TH13) -b*sin(TH12)               -b*sin(TH13);
             0           0          0            0                                      -1                                       0                                       0  a  0  b*sin(TH12)               -a-b*sin(TH12)+b*sin(TH23) -b*sin(TH23);
             0           0          0            0                                       0                                      -1                                       0  0  a  b*sin(TH13)                b*sin(TH23)               -a-b*sin(TH13)-b*sin(TH23);
             (a^2)/(b^3) 0          0           -a/(b^2)                                 0                                       0                                       0  0  0  0                          0                          0;
             0          (a^2)/(b^3) 0            0                                      -a/(b^2)                                 0                                       0  0  0  0                          0                          0;
             0           0         (a^2)/(b^3)   0                                       0                                      -a/(b^2)                                 0  0  0  0                          0                          0;
             0           0          0            a/(b^2)-(1/b)*sin(TH12)-(1/b)*sin(TH13) 0                                       0                                       0  0  0  cos(TH12)+cos(TH13)       -cos(TH12)                 -cos(TH13);
             0           0          0            0                                       a/(b^2)+(1/b)*sin(TH12)-(1/b)*sin(TH23) 0                                       0  0  0 -cos(TH12)                  cos(TH12)+cos(TH23)       -cos(TH23);
             0           0          0            0                                       0                                       a/(b^2)+(1/b)*sin(TH13)+(1/b)*sin(TH23) 0  0  0 -cos(TH13)                 -cos(TH23)                  cos(TH13)+cos(TH23)];


        E = eig(M);
 
        scatter(real(E),imag(E),'.k');
        
    end
end
xlabel('Real','interpreter','latex');
ylabel('Imag','interpreter','latex');
shg;
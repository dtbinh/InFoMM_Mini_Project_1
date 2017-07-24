% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Finds eigenvalues for the N = 4 (discrete) system for different 
%     values of alpha and beta, plotting them on a single viewer.
% INPUT: 
%     : {}
% OUTPUT:
%     : {}

%%
close all;
clear all;
clc;

TH12 = 2*pi/4;
TH13 = 4*pi/4;
TH14 = 6*pi/4;
TH23 = 2*pi/4;
TH24 = 4*pi/4;
TH34 = 2*pi/4;

figure();
hold on;
for a = 1:1:10
    for b = 1:1:10

        M = [0  0  0  0     0  0  0  0    -b  0  0  0    b  0  0  0;
      0  0  0  0     0  0  0  0     0 -b  0  0    0  b  0  0;
      0  0  0  0     0  0  0  0     0  0 -b  0    0  0  b  0;
      0  0  0  0     0  0  0  0     0  0  0 -b    0  0  0  b;
      
      0  0  0  0    -1  0  0  0     a  0  0  0   -a+b*sin(TH12)+b*sin(TH13)+b*sin(TH14) -b*sin(TH12)                           -b*sin(TH13)                           -b*sin(TH14);
      0  0  0  0     0 -1  0  0     0  a  0  0    b*sin(TH12)                           -a-b*sin(TH12)+b*sin(TH23)+b*sin(TH24) -b*sin(TH23)                           -b*sin(TH24);
      0  0  0  0     0  0 -1  0     0  0  a  0    b*sin(TH13)                            b*sin(TH23)                           -a-b*sin(TH13)-b*sin(TH23)+b*sin(TH34) -b*sin(TH34);
      0  0  0  0     0  0  0 -1     0  0  0  a    b*sin(TH14)                            b*sin(TH24)                            b*sin(TH34)                           -a-b*sin(TH14)-b*sin(TH24)-b*sin(TH34);
      
      (a^2)/(b^3) 0          0           0             -a/(b^2) 0       0       0           0  0  0  0     0  0  0  0;
      0          (a^2)/(b^3) 0           0              0      -a/(b^2) 0       0           0  0  0  0     0  0  0  0;
      0           0         (a^2)/(b^3)  0              0       0      -a/(b^2) 0           0  0  0  0     0  0  0  0;
      0           0          0          (a^2)/(b^3)     0       0       0      -a/(b^2)     0  0  0  0     0  0  0  0;                                                    
      
      0  0  0  0     a/(b^2)-(1/b)*sin(TH12)-(1/b)*sin(TH13)-(1/b)*sin(TH14) 0                                                       0                                                       0                                                           0  0  0  0     cos(TH12)+cos(TH13)+cos(TH14) -cos(TH12)                     -cos(TH13)                     -cos(TH14);
      0  0  0  0     0                                                       a/(b^2)+(1/b)*sin(TH12)-(1/b)*sin(TH23)-(1/b)*sin(TH24) 0                                                       0                                                           0  0  0  0    -cos(TH12)                      cos(TH12)+cos(TH23)+cos(TH24) -cos(TH23)                     -cos(TH24);
      0  0  0  0     0                                                       0                                                       a/(b^2)+(1/b)*sin(TH13)+(1/b)*sin(TH23)-(1/b)*sin(TH34) 0                                                           0  0  0  0    -cos(TH13)                     -cos(TH23)                      cos(TH13)+cos(TH23)+cos(TH34) -cos(TH34);
      0  0  0  0     0                                                       0                                                       0                                                       a/(b^2)+(1/b)*sin(TH14)+(1/b)*sin(TH24)+(1/b)*sin(TH34)     0  0  0  0    -cos(TH14)                     -cos(TH24)                      cos(TH34)                      cos(TH14)+cos(TH24)+cos(TH34)];
  

        E = eig(M);
 
        scatter(real(E),imag(E),'.k');
        
    end
end
xlabel('Real','interpreter','latex');
ylabel('Imag','interpreter','latex');
shg;
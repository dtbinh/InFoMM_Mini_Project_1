function [E,E_exp] = linearization_three(a,b)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Finds the eigenvalues of the linearized POLAR formulation of the
%     simple model.
% INPUT: 
%     a: {float} 'alpha' in the model.
%     b: {float} 'beta' in the model.
% OUTPUT:
%     E:   {vec} Eigenvalues of the Polar Jacobian. 

%% Example
% [E] = linearization_three(10,10)

%%

OM = pi/2;
TH12 = 2*pi/3;
TH13 = 4*pi/3;
TH23 = 2*pi/3;

% M1 and M2 below here are for the NON-COMBINED polar system.

M1 = [0           0          0            0                                       0                                       0                                      -b  0  0  b                          0                          0;
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

% M1 here is for the COMBINED polar system, meaning that the dimension is
% now reduced by 1.

% M2 = [0           0          0         0        -b  0  0;
%       0           0          0         0         0 -b  0;
%       0           0         -1         0         a  0  0;
%       0           0          0        -1         0  a  0;
%       (a^2)/(b^3) 0         -2*a/(b^2) 0         0  0  1;
%       0          (a^2)/(b^3) 0        -2*a/(b^2) 0  0 -1;
%       0           0          a/(b^2)  a/(b^2)    0  0  -2];
  
[V1,~] = eig(M1);
E = eig(M1);

M_exp = expm(M1);
[V1M,~] = eig(M_exp);
E_exp = eig(M_exp);

% prod_hold = zeros(25);
% max_hold = zeros(25);
% 
% for a = 1:25
%     for b = 1:25
%         [E] = linearization(a,b);
%         prod_hold(a,b) = prod(real(E));
%         max_hold(a,b) = max(real(E));
%     end
% end
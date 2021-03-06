function [E] = floquet_three(a,b,N)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Finds the approximate integral of the Floquet linearized matrix over
%     an entire period, P.
% INPUT: 
%     a: {float} 'alpha' in the simple model.
%     b: {float} 'beta' in the simple model.
%     N:   {int} Number of matrix-exponential multiplications for the
%                approximated Floquet solution.
% OUTPUT:
%     E:   {vec} Eigenvalues of the approximated integral of the Floquet
%                linearization matrix.

%% Example
% [E] = floquet_three(10,10,1000)

%%
% Period P = 2pi/omega = 2pi/bR = 2pi/b(b^2/a) = 2pia/(b^3)
P = 2*pi*b/a;
P_steps_V = linspace(0,P,N+1);
dt = P_steps_V(2);
P_steps_V = P_steps_V + dt/2;

% Periodic Orbit solutions.
X1_1 = @(t) ((b^2)/(a))*cos(t*a/b);
X1_2 = @(t) ((b^2)/(a))*sin(t*a/b);
X2_1 = @(t) ((b^2)/(a))*cos(t*a/b + 2*pi/3);
X2_2 = @(t) ((b^2)/(a))*sin(t*a/b + 2*pi/3);
X3_1 = @(t) ((b^2)/(a))*cos(t*a/b + 4*pi/3);
X3_2 = @(t) ((b^2)/(a))*sin(t*a/b + 4*pi/3);
V1_1 = @(t) -1*b*sin(t*a/b);
V1_2 = @(t)  b*cos(t*a/b);
V2_1 = @(t) -1*b*sin(t*a/b + 2*pi/3);
V2_2 = @(t)  b*cos(t*a/b + 2*pi/3);
V3_1 = @(t) -1*b*sin(t*a/b + 4*pi/3);
V3_2 = @(t)  b*cos(t*a/b + 4*pi/3);

% Compute and multiply the matrix exponentials.
floq_A = eye(12);
for i = 1:N;
    
    % Choose the particular time-step.
    t = P_steps_V(i);
    
    % Huge definition of the monodromy matrix.
    A_approx = [0 0 0 0 0 0 1 0 0 0 0 0;
                0 0 0 0 0 0 0 1 0 0 0 0;
                0 0 0 0 0 0 0 0 1 0 0 0;
                0 0 0 0 0 0 0 0 0 1 0 0;
                0 0 0 0 0 0 0 0 0 0 1 0;
                0 0 0 0 0 0 0 0 0 0 0 1;
    % Row 7
    a*(X1_1(t))^2/norm([X1_1(t),X1_2(t)])^3 - a/norm([X1_1(t),X1_2(t)]) ...
    a*(X1_1(t)*X1_2(t))/norm([X1_1(t),X1_2(t)])^3 ...
    0 0 0 0 -1 0 ...
    b*(V2_1(t))^2/norm([V2_1(t),V2_2(t)])^3 - b/norm([V2_1(t),V2_2(t)]) ...
    b*(V2_1(t)*V2_2(t))/norm([V2_1(t),V2_2(t)])^3 ...
    b*(V3_1(t))^2/norm([V3_1(t),V3_2(t)])^3 - b/norm([V3_1(t),V3_2(t)]) ...
    b*(V3_1(t)*V3_2(t))/norm([V3_1(t),V3_2(t)])^3;
    % Row 8
    a*(X1_1(t)*X1_2(t))/norm([X1_1(t),X1_2(t)])^3 ...
    a*(X1_2(t))^2/norm([X1_1(t),X1_2(t)])^3 - a/norm([X1_1(t),X1_2(t)]) ...
    0 0 0 0 0 -1 ...
    b*(V2_1(t)*V2_2(t))/norm([V2_1(t),V2_2(t)])^3 ...
    b*(V2_2(t))^2/norm([V2_1(t),V2_2(t)])^3 - b/norm([V2_1(t),V2_2(t)]) ...
    b*(V3_1(t)*V3_2(t))/norm([V3_1(t),V3_2(t)])^3 ...
    b*(V3_2(t))^2/norm([V3_1(t),V3_2(t)])^3 - b/norm([V3_1(t),V3_2(t)]);
    % Row 9
    0 0 a*(X2_1(t))^2/norm([X2_1(t),X2_2(t)])^3 - a/norm([X2_1(t),X2_2(t)]) ...
    a*(X2_1(t)*X2_2(t))/norm([X2_1(t),X2_2(t)])^3 0 0 ...
    b*(V1_1(t))^2/norm([V1_1(t),V1_2(t)])^3 - b/norm([V1_1(t),V1_2(t)]) ...
    b*(V1_1(t)*V1_2(t))/norm([V1_1(t),V1_2(t)])^3 -1 0 ...
    b*(V3_1(t))^2/norm([V3_1(t),V3_2(t)])^3 - b/norm([V3_1(t),V3_2(t)]) ...
    b*(V3_1(t)*V3_2(t))/norm([V3_1(t),V3_2(t)])^3;
    % Row 10
    0 0 a*(X2_1(t)*X2_2(t))/norm([X2_1(t),X2_2(t)])^3 ...
    a*(X2_2(t))^2/norm([X2_1(t),X2_2(t)])^3 - a/norm([X2_1(t),X2_2(t)]) 0 0 ...
    b*(V1_1(t)*V1_2(t))/norm([V1_1(t),V1_2(t)])^3 ...
    b*(V1_2(t))^2/norm([V1_1(t),V1_2(t)])^3 - b/norm([V1_1(t),V1_2(t)]) 0 -1 ...
    b*(V3_1(t)*V3_2(t))/norm([V3_1(t),V3_2(t)])^3 ...
    b*(V3_2(t))^2/norm([V3_1(t),V3_2(t)])^3 - b/norm([V3_1(t),V3_2(t)]);
    % Row 11
    0 0 0 0 a*(X3_1(t))^2/norm([X3_1(t),X3_2(t)])^3 - a/norm([X3_1(t),X3_2(t)]) ...
    a*(X3_1(t)*X3_2(t))/norm([X3_1(t),X3_2(t)])^3 ...
    b*(V1_1(t))^2/norm([V1_1(t),V1_2(t)])^3 - b/norm([V1_1(t),V1_2(t)]) ...
    b*(V1_1(t)*V1_2(t))/norm([V1_1(t),V1_2(t)])^3 ...
    b*(V2_1(t))^2/norm([V2_1(t),V2_2(t)])^3 - b/norm([V2_1(t),V2_2(t)]) ...
    b*(V2_1(t)*V2_2(t))/norm([V2_1(t),V2_2(t)])^3 -1 0;
    % Row 12
    0 0 0 0 a*(X3_1(t)*X3_2(t))/norm([X3_1(t),X3_2(t)])^3 ...
    a*(X3_2(t))^2/norm([X3_1(t),X3_2(t)])^3 - a/norm([X3_1(t),X3_2(t)]) ...
    b*(V1_1(t)*V1_2(t))/norm([V1_1(t),V1_2(t)])^3 ...
    b*(V1_2(t))^2/norm([V1_1(t),V1_2(t)])^3 - b/norm([V1_1(t),V1_2(t)]) ...
    b*(V2_1(t)*V2_2(t))/norm([V2_1(t),V2_2(t)])^3 ...
    b*(V2_2(t))^2/norm([V2_1(t),V2_2(t)])^3 - b/norm([V2_1(t),V2_2(t)]) 0 -1];

%     A_approx = [-t  0  0  0  0  0  0  0;
%                  0 -t  0  0  0  0  0  0;
%                  0  0 -t  0  0  0  0  0;
%                  0  0  0 -t  0  0  0  0;
%                  0  0  0  0 -t  0  0  0;
%                  0  0  0  0  0 -t  0  0;
%                  0  0  0  0  0  0 -t  0;
%                  0  0  0  0  0  0  0 -t];

    floq_A = expm((A_approx)*dt)*floq_A;
    if i == N
        A_eig = eig(A_approx)
    end
end

floq = floq_A
E = eig(floq_A);

%%

pos = [-1 -1 2 2];
rectangle('Position',pos,'Curvature',[1 1])
axis equal
hold on
scatter(real(E),imag(E),'xr')
xlabel('Real','interpreter','latex')
ylabel('Imag','interpreter','latex')
shg
   
end

% for i = 1:0.1:5
%     [E] = floquet_three(10^i,10,1000);
% end
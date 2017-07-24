function fn = amended_N(t,y)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     General form for (DM2) using ode45 and other solvers. Choose N <= 10.
% INPUT: 
%     t: {vec} Timestep (run through chosen solver).
%     y: {vec} Vector of [positions,velocities].
% OUTPUT:
%      : {}

%% Example
% N = 4; y = [rand(1,4*N),zeros(1,2*N*N)]; 
% [t,u]= ode45(@amended_N ,[0 ,1000], y);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% data = u;
% shg;

%%
format long;
a=10; b=10; g=0.02;
[n,m]=size(y);
% This assumes that a random initialisation will NOT choose any zero
% values
M = max(n,m);
N = -1 + sqrt(16+8*M)/4;
fn= zeros(n,m);
for i = 1:2*N
    fn(i) = y(i+2*N);
end


% Reshape the previous position
r_prev_A = reshape(y((4*N+1):end)',2*N,N)';

pos_V = y(1:2*N);
pos_V = reshape(pos_V,2,N)';
r_unit_A = direction_finder(pos_V);

r_diff_A = r_unit_A - r_prev_A;
r_factor_A = zeros(N);
for i = 1:(N-1)
    for j = (i+1):N
        r_factor_A(i,j) = ...
        1*norm([r_diff_A(i,2*j-1),r_diff_A(i,2*j)]);
    end
end
% This builds the array of ||r(i,t)@(t) - r(i,j)@(t-1)||.
r_factor_A = r_factor_A + r_factor_A';
r_prev_A = r_unit_A;
R = [];
for i = 1:N
    R = [R,r_factor_A(:,i),r_factor_A(:,i)];
end
r_factor_A = R;
gamma_A = r_factor_A.*r_unit_A;
% This vector is to counteract any closeness between drones.
gamma_V = sum(gamma_A);

% Now to update the previous r_ij.
r_unit_conc_V = reshape(r_unit_A',(M-4*N),1)';
for i = (4*N+1):M
    fn(i) = r_unit_conc_V(i-4*N);
end

b_func1 = 0;
b_func2 = 0;
for i = 1:N
    b_func1 = b_func1 - b*y(2*i-1+2*N)/norm([y(2*i-1+2*N),y(2*i+2*N)]);
    b_func2 = b_func2 - b*y(2*i+2*N)/norm([y(2*i-1+2*N),y(2*i+2*N)]);
end

for i = 1:N
    fn(2*i-1+2*N) = -a*y(2*i-1)/norm([y(2*i-1),y(2*i)]) ...
             + b_func1 + b*y(2*i-1+2*N)/norm([y(2*i-1+2*N),y(2*i+2*N)]) ...
             - g*gamma_V(2*i-1) - y(2*i-1+2*N);
    fn(2*i+2*N) = -a*y(2*i)/norm([y(2*i-1),y(2*i)]) ...
             + b_func2 + b*y(2*i+2*N)/norm([y(2*i-1+2*N),y(2*i+2*N)]) ...
             - g*gamma_V(2*i) - y(2*i+2*N);
end

return

%% EXAMPLES

% Basic simulation, ode45
% 
% N = 4;
% y = rand(1,4*N);
% [t,u]= ode45(@basic_N ,[0 ,500], y);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% shg;

% Basic simulation, ode23
%
% N = 4;
% y = rand(1,4*N);
% [t,u]= ode15s(@basic_N ,[0 ,500], y);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% shg;

% Forcing one in the wrong direction.
% 
% N = 4;
% dro_pos_A = 5*randn(N,2);
% y_unit_A = target_finder(dro_pos_A,[0,0]);
% dro_vel_A = ([0, -1; 1 0]*y_unit_A')';
% dro_vel_A(N,:) = -10*dro_vel_A(N,:);
% initial_cond_V = [reshape(dro_pos_A',1,2*N),reshape(dro_vel_A',1,2*N)];
% [t,u]= ode45(@basic_N ,[0 ,1000], initial_cond_V);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% shg;

% Some start outside the radius
%
% N = 4;
% [t,u]= ode45(@basic_N ,[0 ,1000], diag(diag(10*ones))*randn(1,2*N));
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% shg;

% Forward Euler
%
% N = 4;
% [t,u]= feuler(@basic_N ,[0 ,100], diag(diag(10*ones))*randn(1,2*N), 20000);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% shg;

% Backward Euler
%
% N = 4;
% [t,u]= beuler(@basic_N ,[0 ,100], diag(diag(10*ones))*randn(1,2*N), 2000);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% shg;

end
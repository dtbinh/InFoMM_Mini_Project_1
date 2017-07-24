function fn = basic_N(t,y)

% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     General form for (DM1) using ode45 and other solvers. Choose N <= 10.
% INPUT: 
%     t: {vec} Timestep (run through chosen solver).
%     y: {vec} Vector of [positions,velocities].
% OUTPUT:
%      : {}

%% Example
% N = 4; y = rand(1,4*N); [t,u]= ode45(@basic_N ,[0 ,500], y);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% data = u;
% shg;

%%
format long;
a=10; b=10;
[n,m]=size(y);
N = max(n,m)/4;
fn= zeros(n,m);

for i = 1:2*N
    fn(i) = y(i+2*N);
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
             - y(2*i-1+2*N);
    fn(2*i+2*N) = -a*y(2*i)/norm([y(2*i-1),y(2*i)]) ...
             + b_func2 + b*y(2*i+2*N)/norm([y(2*i-1+2*N),y(2*i+2*N)]) ...
             - y(2*i+2*N);
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

%Same Direction
% 
% N = 4;
% dro_pos_A = 5*randn(N,2);
% y_unit_A = target_finder(dro_pos_A,[0,0]);
% dro_vel_A = ([0, -1; 1 0]*y_unit_A')';
% initial_cond_V = [reshape(dro_pos_A',1,2*N),reshape(dro_vel_A',1,2*N)];
% [t,u]= ode45(@basic_N ,[0 ,1000], initial_cond_V);
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
% dro_vel_A(N,:) = -5*dro_vel_A(N,:);
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
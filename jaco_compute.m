function [A,V,E] = jaco_compute(a,b,theta)

A = [0 0 0 0 1 0 0 0;
 0 0 0 0 0 1 0 0;
 0 0 0 0 0 0 1 0;
 0 0 0 0 0 0 0 1;
 (a^2)/(b^2)*(cos(theta)^2 - 1) (a^2)/(b^2)*cos(theta)*sin(theta) 0 0 -1 0 -1+sin(theta)^2 -sin(theta)*cos(theta);
 (a^2)/(b^2)*cos(theta)*sin(theta) (a^2)/(b^2)*(sin(theta)^2 - 1) 0 0 0 -1 -sin(theta)*cos(theta) -1+cos(theta)^2;
 0 0 (a^2)/(b^2)*(cos(theta)^2 - 1) (a^2)/(b^2)*cos(theta)*sin(theta) -1+sin(theta)^2 -sin(theta)*cos(theta) -1 0;
 0 0 (a^2)/(b^2)*cos(theta)*sin(theta) (a^2)/(b^2)*(sin(theta)^2 - 1) -sin(theta)*cos(theta) -1+cos(theta)^2 0 -1];
% 
% N = 1000;
% figure();
% pos = [-1 -1 2 2];
% rectangle('Position',pos,'Curvature',[1 1]);
% axis equal;
% hold on;
% shg;
% max_eig = zeros(N+1,1);
% prod_eig = zeros(N+1,1);
% rads = linspace(0,2*pi,N);
% for i = 1:N
%     rads(i)
%     [A,~,E] = jaco_compute(a,b,rads(i));
%     max_eig(i) = max(real(E));
%     prod_eig(i) = prod(eig(A));
%     eig_points = scatter(real(E),imag(E));
%     drawnow;
%     delete(eig_points);
%     shg;
% end
% 
% plot(max_eig)
% legend('Max_eig')
% figure()
% plot(real(prod_eig))
% legend('Prod_eig')
% shg

[V,~] = eig(A);
[E] = eig(A);

% Parameter sweep over a and/or b.
% 
% close all;
% N = 1000;
% a = 10;
% b = 10;
% figure();
% pos = [-1 -1 2 2];
% rectangle('Position',pos,'Curvature',[1 1]);
% axis equal;
% hold on;
% shg;
% max_eig = zeros(N+1,1);
% prod_eig = zeros(N+1,1);
% alphas = linspace(1,20,N);
% betas = linspace(1,20,N);
% for i = 1:N
%     i
%     [A,~,E] = jaco_compute(a,betas(i),0);
%     gersh = [sum(abs(A(5,:)))-1;sum(abs(A(6,:)))-1;sum(abs(A(7,:)))-1;...
%              sum(abs(A(8,:)))-1];
%     max_eig(i) = max(real(E));
%     prod_eig(i) = prod(eig(A));
%     eig_points = scatter(real(E),imag(E));
%     gersh1 = [-1-gersh(1) -gersh(1) 2*gersh(1) 2*gersh(1)];
%     gersh2 = [-1-gersh(2) -gersh(2) 2*gersh(2) 2*gersh(2)];
%     gersh1rect = rectangle('Position',gersh1,'Curvature',[1 1]);
%     gersh2rect = rectangle('Position',gersh2,'Curvature',[1 1]);
%     drawnow;
%     delete(eig_points);
%     delete(gersh2rect);
%     shg;
% end
% 
% plot(max_eig)
% legend('Max_eig')
% figure()
% plot(real(prod_eig))
% legend('Prod_eig')
% shg
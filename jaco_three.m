function [] = jaco_three

% ONE OF THE EIGENVALUES BECOMES POSITIVE, SO THERE MAY BE A MISTAKE IN THE
% A_APPROX MATRIX.

close all;

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

a = 10;
b = 10;
N = 10000;
figure();
pos = [-1 -1 2 2];
rectangle('Position',pos,'Curvature',[1 1]);
axis equal;
hold on;
shg;
max_eig = zeros(N+1,1);
prod_eig = zeros(N+1,1);
rads = linspace(0,2*pi,N);
for i = 1:N
    t = rads(i)
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
    b*(V3_1(t))^2/norm([V3_1(t),V3_2(t)])^3 - b/norm([V3_1(t),V3_2(t)]) ...
    b*(V3_1(t)*V3_2(t))/norm([V3_1(t),V3_2(t)])^3;
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
    b*(V3_1(t))^2/norm([V3_1(t),V3_2(t)])^3 - b/norm([V3_1(t),V3_2(t)]) ...
    b*(V3_1(t)*V3_2(t))/norm([V3_1(t),V3_2(t)])^3;
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
    E = eig(A_approx);
    max_eig(i) = max(real(E));
    prod_eig(i) = prod(E);
    eig_points = scatter(real(E),imag(E));
    drawnow;
    delete(eig_points);
    shg;
end

plot(max_eig)
legend('Max_eig')
figure()
plot(real(prod_eig))
legend('Prod_eig')
shg

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
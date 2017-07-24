% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is just to make sure that dr/dt is equal to d(theta)/dt.
% INPUT: 
%     T:   {int} Total amount of time to run the visualisation.
%    dt: {float} Timestep to be taken.
% OUTPUT:
%      : {}

%%

close all; clear all; format compact; clc;

T = 1001;

x1 = [0,0];
time_V = linspace(0,1,T);
places = [0,0];
angles = [0];
dist = [0,0];

for i = 1:T
    places(end+1,:) = [10,4] + time_V(i)*[-18,-6];
end

places = places(2:end,:);
DIST = places - x1;
angles = atan(DIST(:,1)./DIST(:,2));

for i = 1:T
    dist(end+1,:) = DIST(i,:)/norm(DIST(i,:));
end
dist = dist(2:end,:);

traj = figure();
plot(places(:,1),places(:,2));
hold on
scatter(x1(1), x1(2), T-1, 'rx');
movegui(traj, 'east');
shg;

ang_diff = zeros(T-1,1);
vec_diff = zeros(T-1,1);
for i = 2:T
    ang_diff(i-1) = abs(angles(i) - angles(i-1));
    if ang_diff(i-1) > 1;
        ang_diff(i-1) = abs(ang_diff(i-1) - pi);
    end
    vec_diff(i-1) = norm(dist(i,:) - dist(i-1,:));
end

% ang_diff = ang_diff/max(ang_diff);
% vec_diff = vec_diff/max(vec_diff);

angfig = figure();
plot(ang_diff);
hold on
plot(vec_diff);
legend('ang_diff','vec_diff')

function [] = reflection_distance_model(sq,r,k)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     From a random initialisation of N drones, we use a 'nearest
%     neighbour' approach, using the 1/(r^2) drop-off rate of signal
%     amplitude, to settle the configuration of the system.
% INPUT: 
%    sq:   {int} Square root of the number of drones in the system, < 10.
%     r: {float} Radius of the drones.
%     k:   {int} Number of nearest neighbours to react to.
% OUTPUT:
%     : {}

%% Example
% [] = reflection_distance_model(10,1,4)

%%
% sq = 10; r = 1; k = 8;
keepvars = {'sq','r','k'};
clearvars('-except', keepvars{:});close all; clc; format compact;

% Total number of drones in the system.
N = sq^2;

% KNN matches the drone with itself perfectly.
k = k + 1;

% Plot drones on a sq x sq lattice.
position_vec = [];
movement_flag = [];
for i = 1:sq
    for j = 1:sq
        if i ~= 1 && i ~= sq && j ~= 1 && j ~= sq
            % Place drones on a noisy lattice.
            position_vec(end+1,:) = [i,j] + 0.5*rand(1,2);
            % Place drones on a rigid lattice.
            % position_vec(end+1,:) = [i,j];
            movement_flag(end+1) = 1;
        else
            % Keep the outer domain rigid, and do not update positions.
            position_vec(end+1,:) = [i,j];
            movement_flag(end+1) = 0;
        end
    end
end

% Plot the drones, to make sure that there is no overlap.
figure();
hold on;
colour = linspace(1,10,N);
scatter(position_vec(:,1),position_vec(:,2),100*r,colour);
axis([0 11 0 11]);
shg;

%%
curr_pos_vec = position_vec;
new_pos_vec = zeros(N,2);

% KNN matches the drone with itself perfectly.
k = k + 1;
[idx_array,d] = knnsearch(curr_pos_vec, curr_pos_vec, 'k', k);

% Plot the vectors to the closest neighbours for a random subset.
ran_subset = ceil(N*rand(sq,1));
for i = 1:sq
    p = ran_subset(i);
    quiver(repmat(curr_pos_vec(p,1),1,k),repmat(curr_pos_vec(p,2),1,k),...
        curr_pos_vec(idx_array(p,:),1) - repmat(curr_pos_vec(p,1),1,k),...
        curr_pos_vec(idx_array(p,:),2) - repmat(curr_pos_vec(p,2),1,k));
end
shg;

%%

ran_subset = ceil(N*rand(sq,1));
v = ran_subset(1);
z = 1;
while movement_flag(v) == 0
    z = z + 1;
    v = ran_subset(z);
end

for t = 1:1000
    if mod(t,50) == 0
        t
    end
    for i = 1:N
        if movement_flag(i) == 1
            ind = idx_array(i,:);
            pts = position_vec(ind,:);
            % Assume that our estimate of distance from the inverse-square
            % law is not great, so we will have errors.
            pts = pts - curr_pos_vec(i,:) + 0.1*randn(k,1);
            
            % Check if drones are too close, then they repel.
%             for j = 1:k
%                 if norm(pts(j,:)) < 1
%                     pts(j,:) = -pts(j,:)/norm(pts(j,:));
%                 end
%             end   
            
            % Maybe this is a better form? Normalise all distance vectors
            % FIRST and penalize vectors that are short.
            pts_norm = zeros(k,1);
            for j = 1:k
                pts_norm(j) = norm(pts(j,:));
            end
            pts = -pts./(pts_norm).^2;
            
            % Compute the average
            move_vec = sum(pts,1)/k;
            
            if i == v
                v_to_plot = pts;
                v_move = move_vec;
            end
            
            new_pos_vec(i,:) = curr_pos_vec(i,:) + 0.01*move_vec;
        else
            new_pos_vec(i,:) = curr_pos_vec(i,:);
        end
    end

    curr_pos_vec = new_pos_vec;

    %%

    clf;
    hold on;
    colour = linspace(1,10,N);
    scatter(curr_pos_vec(:,1),curr_pos_vec(:,2),100*r,colour);
    axis([0 11 0 11]);

    % Perform KNN.
    [idx_array,d] = knnsearch(curr_pos_vec, curr_pos_vec, 'k', k);

    % Plot the vectors to the closest neighbours for a random subset.
    for i = 1:sq
        p = ran_subset(i);
        quiver(repmat(curr_pos_vec(p,1),1,k),repmat(curr_pos_vec(p,2),1,k),...
            curr_pos_vec(idx_array(p,:),1) - repmat(curr_pos_vec(p,1),1,k),...
            curr_pos_vec(idx_array(p,:),2) - repmat(curr_pos_vec(p,2),1,k));
    end
%         quiver(repmat(curr_pos_vec(v,1),1,k),repmat(curr_pos_vec(v,2),1,k),...
%             v_to_plot(:,1)',v_to_plot(:,2)');
        MV = quiver(curr_pos_vec(v,1),curr_pos_vec(v,2),...
                v_move(1),v_move(2));
        set(MV,'linewidth',1);
        set(MV,'color',[0,0,0]);
        
    drawnow;
end
        
% n = floor(log10(A));

function [lines_to_draw] = plot_maker(N,T,dt,P,Q,R,V,Y1,Y2,t_vec,alpha,...
                                      beta)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is a very boring if/else statement to determine how many
%     trajectories to make, as long as N < 10. To Oliver, I apologise for
%     this monstrosity, but I have no idea how to adaptively set the number
%     of plots to make in Matlab.
% INPUT: 
%     N:   {int} Number of drones in the model.
%     T:   {int} Number of data points wanted in the simulation.
%    dt: {float} Length of timestep in the simulation.
%     P: {array} Initial drone positions.
%     Q: {array} Initial drone velocities.
%     R:   {vec} Target position vector.
%     V:   {vec} Unit array of drone orientations.
%    Y1:  {func} Target x-coordinate function.
%    Y2:  {func} Target y-coordinate function.
% t_vec:   {vec} Complete vector of timesteps.
% alpha: {float} 'alpha' in the original system equations.
%  beta: {float} 'beta' in the original system equations.
%    
% OUTPUT:
%     : {}

%% Examples
% [] = plot_maker(4,5001,0.02,D_pos_array,D_vel_array,T_pos_vec,...
%                 v_unit_array,Y1,Y2,all_time,1.5,0.5)

%%

D_traj_array = zeros(T,2*N);
D_dir_array = zeros(T,2*N);
D_traj_array(1,:) = reshape(P',1,2*N);
D_dir_array(1,:) = reshape(Q',1,2*N);
D_pos_previous = D_traj_array(1,:);
D_vel_previous = D_dir_array(1,:);
T_trajectory = [Y1(t_vec);Y2(t_vec)]';

if N == 1
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1')
    shg;
    
    v_sum_vec = zeros(T,2);

    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));

        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));

        drawnow;
    end
    
elseif N == 2
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);
        
        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));

        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        
        drawnow;
    end
    
elseif N == 3
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D3 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);
    line_D3 = animatedline('Color','m','MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2','Drone 3')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(D3,D_traj_array(t,5),D_traj_array(t,6));

        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(line_D3,D_traj_array(t,5),D_traj_array(t,6));

        drawnow;
    end
    
elseif N == 4
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D3 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    D4 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);
    line_D3 = animatedline('Color','m','MaximumNumPoints',200);
    line_D4 = animatedline('Color','k','MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(D4,D_traj_array(t,7),D_traj_array(t,8));


        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(line_D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(line_D4,D_traj_array(t,7),D_traj_array(t,8));

        drawnow;
    end
    
elseif N == 5
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D3 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    D4 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    D5 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);
    line_D3 = animatedline('Color','m','MaximumNumPoints',200);
    line_D4 = animatedline('Color','k','MaximumNumPoints',200);
    line_D5 = animatedline('Color','r','LineStyle','--',...
        'MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(D5,D_traj_array(t,9),D_traj_array(t,10));


        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(line_D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(line_D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(line_D5,D_traj_array(t,9),D_traj_array(t,10));

        drawnow;
    end
    
elseif N == 6
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D3 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    D4 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    D5 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    D6 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);
    line_D3 = animatedline('Color','m','MaximumNumPoints',200);
    line_D4 = animatedline('Color','k','MaximumNumPoints',200);
    line_D5 = animatedline('Color','r','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D6 = animatedline('Color','b','LineStyle','--',...
        'MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4',...
        'Drone 5','Drone 6')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(D6,D_traj_array(t,11),D_traj_array(t,12));


        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(line_D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(line_D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(line_D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(line_D6,D_traj_array(t,11),D_traj_array(t,12));

        drawnow;
    end
    
elseif N == 7
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D3 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    D4 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    D5 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    D6 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D7 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);
    line_D3 = animatedline('Color','m','MaximumNumPoints',200);
    line_D4 = animatedline('Color','k','MaximumNumPoints',200);
    line_D5 = animatedline('Color','r','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D6 = animatedline('Color','b','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D7 = animatedline('Color','g','LineStyle','--',...
        'MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4',...
        'Drone 5','Drone 6','Drone 7')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(D6,D_traj_array(t,11),D_traj_array(t,12));
        addpoints(D7,D_traj_array(t,13),D_traj_array(t,14));


        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(line_D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(line_D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(line_D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(line_D6,D_traj_array(t,11),D_traj_array(t,12));
        addpoints(line_D7,D_traj_array(t,13),D_traj_array(t,14));

        drawnow;
    end
    
elseif N == 8
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D3 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    D4 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    D5 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    D6 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D7 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D8 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);
    line_D3 = animatedline('Color','m','MaximumNumPoints',200);
    line_D4 = animatedline('Color','k','MaximumNumPoints',200);
    line_D5 = animatedline('Color','r','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D6 = animatedline('Color','b','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D7 = animatedline('Color','g','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D8 = animatedline('Color','m','LineStyle','--',...
        'MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4',...
        'Drone 5','Drone 6','Drone 7','Drone 8')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(D6,D_traj_array(t,11),D_traj_array(t,12));
        addpoints(D7,D_traj_array(t,13),D_traj_array(t,14));
        addpoints(D8,D_traj_array(t,15),D_traj_array(t,16));


        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(line_D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(line_D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(line_D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(line_D6,D_traj_array(t,11),D_traj_array(t,12));
        addpoints(line_D7,D_traj_array(t,13),D_traj_array(t,14));
        addpoints(line_D8,D_traj_array(t,15),D_traj_array(t,16));

        drawnow;
    end
    
elseif N == 9
    traj_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D2 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D3 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    D4 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    D5 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    D6 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    D7 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    D8 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    D9 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);
    line_D2 = animatedline('Color','g','MaximumNumPoints',200);
    line_D3 = animatedline('Color','m','MaximumNumPoints',200);
    line_D4 = animatedline('Color','k','MaximumNumPoints',200);
    line_D5 = animatedline('Color','r','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D6 = animatedline('Color','b','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D7 = animatedline('Color','g','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D8 = animatedline('Color','m','LineStyle','--',...
        'MaximumNumPoints',200);
    line_D9 = animatedline('Color','k','LineStyle','--',...
        'MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4',...
        'Drone 5','Drone 6','Drone 7','Drone 8','Drone 9')
    shg;
    
    for t = 2:T

        % Reshape the arrays.
        D_pos_array = reshape(D_pos_previous,2,N)';
        D_vel_array = reshape(D_vel_previous,2,N)';
        T_pos_vec = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_array] = direction_finder(D_pos_array);
        [v_unit_array] = orientation_finder(D_vel_array);
        [y_unit_array] = target_finder(D_pos_array,T_pos_vec);
        vec_repulsion = sum(R,1);
        v_sum_vec(t,:) = vec_repulsion;

        % Update the drone trajectories.
        D_traj_array(t,:) = D_pos_previous + D_vel_previous*dt;
        D_pos_previous = D_traj_array(t,:);  

        % Update the drone velocities.
        D_dir_array(t,:) = D_vel_previous + ...
            (alpha*reshape(y_unit_array',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_array',1,2*N) - ...
            D_vel_previous)*dt;
        D_vel_previous = D_dir_array(t,:);

        % Update the drone positions.
        addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(D6,D_traj_array(t,11),D_traj_array(t,12));
        addpoints(D7,D_traj_array(t,13),D_traj_array(t,14));
        addpoints(D8,D_traj_array(t,15),D_traj_array(t,16));
        addpoints(D9,D_traj_array(t,17),D_traj_array(t,18));


        % Add new points to the trajectory lines.
        addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
        addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
        addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
        addpoints(line_D3,D_traj_array(t,5),D_traj_array(t,6));
        addpoints(line_D4,D_traj_array(t,7),D_traj_array(t,8));
        addpoints(line_D5,D_traj_array(t,9),D_traj_array(t,10));
        addpoints(line_D6,D_traj_array(t,11),D_traj_array(t,12));
        addpoints(line_D7,D_traj_array(t,13),D_traj_array(t,14));
        addpoints(line_D8,D_traj_array(t,15),D_traj_array(t,16));
        addpoints(line_D9,D_traj_array(t,17),D_traj_array(t,18));

        drawnow;
    end
    
else
    sprintf('No plots will be made. N >= 10')
end
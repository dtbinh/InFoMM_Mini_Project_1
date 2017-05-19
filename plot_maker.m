function [] = plot_maker(N,T,dt,P,Q,R,V,Y1,Y2,t_vec,alpha,beta)

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

%% Example
% [] = plot_maker(4,5001,0.02,dro_pos_A,dro_vel_A,tar_pos_V,...
%                 v_unit_A,Y1,Y2,all_time_V,1.5,0.5)

%%

dro_traj_A = zeros(T,2*N);
dro_direc_A = zeros(T,2*N);
dro_traj_A(1,:) = reshape(P',1,2*N);
dro_direc_A(1,:) = reshape(Q',1,2*N);
dro_pos_prev_V = dro_traj_A(1,:);
dro_vel_prev_V = dro_direc_A(1,:);
tar_traj_V = [Y1(t_vec);Y2(t_vec)]';

if N == 1
    trajectories_plot = figure();

    % Set up the animation to view the trajectories.
    T1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    D1 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    line_T1 = animatedline('Color','r','MaximumNumPoints',200);
    line_D1 = animatedline('Color','b','MaximumNumPoints',200);

    axis([-3,3,-3,3]);
    legend('Target','Drone 1')
    shg;
    
    v_sum_V = zeros(T,2);

    for t = 2:T

        % Reshape the arrays.
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        r_unit_A = direction_finder(dro_pos_A);
        v_unit_A = orientation_finder(dro_vel_A);
        y_unit_A = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));

        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));

        drawnow;
    end
    
elseif N == 2
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);
        
        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));

        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        
        drawnow;
    end
    
elseif N == 3
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(D3,dro_traj_A(t,5),dro_traj_A(t,6));

        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(line_D3,dro_traj_A(t,5),dro_traj_A(t,6));

        drawnow;
    end
    
elseif N == 4
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(D4,dro_traj_A(t,7),dro_traj_A(t,8));


        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(line_D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(line_D4,dro_traj_A(t,7),dro_traj_A(t,8));

        drawnow;
    end
    
elseif N == 5
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(D5,dro_traj_A(t,9),dro_traj_A(t,10));


        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(line_D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(line_D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(line_D5,dro_traj_A(t,9),dro_traj_A(t,10));

        drawnow;
    end
    
elseif N == 6
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(D6,dro_traj_A(t,11),dro_traj_A(t,12));


        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(line_D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(line_D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(line_D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(line_D6,dro_traj_A(t,11),dro_traj_A(t,12));

        drawnow;
    end
    
elseif N == 7
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(D6,dro_traj_A(t,11),dro_traj_A(t,12));
        addpoints(D7,dro_traj_A(t,13),dro_traj_A(t,14));


        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(line_D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(line_D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(line_D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(line_D6,dro_traj_A(t,11),dro_traj_A(t,12));
        addpoints(line_D7,dro_traj_A(t,13),dro_traj_A(t,14));

        drawnow;
    end
    
elseif N == 8
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(D6,dro_traj_A(t,11),dro_traj_A(t,12));
        addpoints(D7,dro_traj_A(t,13),dro_traj_A(t,14));
        addpoints(D8,dro_traj_A(t,15),dro_traj_A(t,16));


        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(line_D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(line_D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(line_D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(line_D6,dro_traj_A(t,11),dro_traj_A(t,12));
        addpoints(line_D7,dro_traj_A(t,13),dro_traj_A(t,14));
        addpoints(line_D8,dro_traj_A(t,15),dro_traj_A(t,16));

        drawnow;
    end
    
elseif N == 9
    trajectories_plot = figure();

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
        dro_pos_A = reshape(dro_pos_prev_V,2,N)';
        dro_vel_A = reshape(dro_vel_prev_V,2,N)';
        tar_pos_V = [Y1(t_vec(t)),Y2(t_vec(t))];

        % Recompute all unit vectors.
        [r_unit_A] = direction_finder(dro_pos_A);
        [v_unit_A] = orientation_finder(dro_vel_A);
        [y_unit_A] = target_finder(dro_pos_A,tar_pos_V);
        vec_repulsion = sum(R,1);
        v_sum_V(t,:) = vec_repulsion;

        % Update the drone trajectories.
        dro_traj_A(t,:) = dro_pos_prev_V + dro_vel_prev_V*dt;
        dro_pos_prev_V = dro_traj_A(t,:);  

        % Update the drone velocities.
        dro_direc_A(t,:) = dro_vel_prev_V + ...
            (alpha*reshape(y_unit_A',1,2*N) - ...
            beta*repmat(vec_repulsion,1,N) + ...
            beta*reshape(v_unit_A',1,2*N) - ...
            dro_vel_prev_V)*dt;
        dro_vel_prev_V = dro_direc_A(t,:);

        % Update the drone positions.
        addpoints(T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(D6,dro_traj_A(t,11),dro_traj_A(t,12));
        addpoints(D7,dro_traj_A(t,13),dro_traj_A(t,14));
        addpoints(D8,dro_traj_A(t,15),dro_traj_A(t,16));
        addpoints(D9,dro_traj_A(t,17),dro_traj_A(t,18));


        % Add new points to the trajectory lines.
        addpoints(line_T1,tar_traj_V(t,1),tar_traj_V(t,2));
        addpoints(line_D1,dro_traj_A(t,1),dro_traj_A(t,2));
        addpoints(line_D2,dro_traj_A(t,3),dro_traj_A(t,4));
        addpoints(line_D3,dro_traj_A(t,5),dro_traj_A(t,6));
        addpoints(line_D4,dro_traj_A(t,7),dro_traj_A(t,8));
        addpoints(line_D5,dro_traj_A(t,9),dro_traj_A(t,10));
        addpoints(line_D6,dro_traj_A(t,11),dro_traj_A(t,12));
        addpoints(line_D7,dro_traj_A(t,13),dro_traj_A(t,14));
        addpoints(line_D8,dro_traj_A(t,15),dro_traj_A(t,16));
        addpoints(line_D9,dro_traj_A(t,17),dro_traj_A(t,18));

        drawnow;
    end
    
else
    sprintf('No plots will be made. N >= 10')
end
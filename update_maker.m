function [] = update_maker(N)

% Author: Joseph Field 
% Date:   May 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     This is a very boring if/else statement to continue making the plots,
%     as long as N < 10. To Oliver, I apologise for this monstrosity, but I
%     have no idea how to adaptively set the number of plots to make in 
%     Matlab.
% INPUT: 
%     N: {int} Number of drones in the model.
% OUTPUT:
%     : {}

%% Example
% [] = update_maker(6)

%%

if N == 1
    % Update the drone positions.
    addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
    addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));

    % Add new points to the trajectory lines.
    addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
    addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
    
elseif N == 2
    % Update the drone positions.
    addpoints(T1,T_trajectory(t,1),T_trajectory(t,2));
    addpoints(D1,D_traj_array(t,1),D_traj_array(t,2));
    addpoints(D2,D_traj_array(t,3),D_traj_array(t,4));

    % Add new points to the trajectory lines.
    addpoints(line_T1,T_trajectory(t,1),T_trajectory(t,2));
    addpoints(line_D1,D_traj_array(t,1),D_traj_array(t,2));
    addpoints(line_D2,D_traj_array(t,3),D_traj_array(t,4));
    
elseif N == 3
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
    
elseif N == 4
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
    
elseif N == 5
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
    
elseif N == 6
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
    
elseif N == 7
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
    
elseif N == 8
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
    
elseif N == 9
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
    
else
    sprintf('No plots will be made. N >= 10')
end
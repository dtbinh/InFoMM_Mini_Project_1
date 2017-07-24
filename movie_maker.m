% Author: Joseph Field 
% Date:   June 2017.
% Course: Mini Project One, University of Oxford.
%
% DESCRIPTION:
%     Makes the .avi video of the 'basic_N' simulations.
% INPUT: 
%     N: Number of drones in the system (<=10).
% OUTPUT:
%      : {}

%% Example
% N = 10; y = rand(1,4*N); [t,u]= ode45(@basic_N ,[0 ,500], y);
% close all;
% for i = 1:N
%     plot(u(:,2*i-1),u(:,2*i));
%     hold on
% end
% data = u;
% shg;
% movie_maker

%%

ax_size = 20;
[m,n] = size(data);
N = n/4;

if N == 1
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1')
    shg;

    vidObj = VideoWriter('1_drone.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);

elseif N == 2

    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2')
    shg;

    vidObj = VideoWriter('2_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
elseif N == 3
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3')
    shg;

    vidObj = VideoWriter('3_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
elseif N == 4
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);
    h5 = animatedline('Color','k','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4')
    shg;

    vidObj = VideoWriter('4_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));
        addpoints(g5,data(i,7),data(i,8));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));
        addpoints(h5,data(i,7),data(i,8));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
elseif N == 5
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    g6 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);
    h5 = animatedline('Color','k','MaximumNumPoints',10);
    h6 = animatedline('Color','r','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5')
    shg;

    vidObj = VideoWriter('5_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));
        addpoints(g5,data(i,7),data(i,8));
        addpoints(g6,data(i,9),data(i,10));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));
        addpoints(h5,data(i,7),data(i,8));
        addpoints(h6,data(i,9),data(i,10));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
elseif N == 6
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    g6 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    g7 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);
    h5 = animatedline('Color','k','MaximumNumPoints',10);
    h6 = animatedline('Color','r','MaximumNumPoints',10);
    h7 = animatedline('Color','b','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5', ...
        'Drone 6')
    shg;

    vidObj = VideoWriter('6_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));
        addpoints(g5,data(i,7),data(i,8));
        addpoints(g6,data(i,9),data(i,10));
        addpoints(g7,data(i,11),data(i,12));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));
        addpoints(h5,data(i,7),data(i,8));
        addpoints(h6,data(i,9),data(i,10));
        addpoints(h7,data(i,11),data(i,12));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
elseif N == 7
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    g6 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    g7 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g8 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);
    h5 = animatedline('Color','k','MaximumNumPoints',10);
    h6 = animatedline('Color','r','MaximumNumPoints',10);
    h7 = animatedline('Color','b','MaximumNumPoints',10);
    h8 = animatedline('Color','g','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5', ...
        'Drone 6','Drone 7')
    shg;

    vidObj = VideoWriter('7_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));
        addpoints(g5,data(i,7),data(i,8));
        addpoints(g6,data(i,9),data(i,10));
        addpoints(g7,data(i,11),data(i,12));
        addpoints(g8,data(i,13),data(i,14));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));
        addpoints(h5,data(i,7),data(i,8));
        addpoints(h6,data(i,9),data(i,10));
        addpoints(h7,data(i,11),data(i,12));
        addpoints(h8,data(i,13),data(i,14));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
elseif N == 8
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    g6 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    g7 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g8 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g9 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);
    h5 = animatedline('Color','k','MaximumNumPoints',10);
    h6 = animatedline('Color','r','MaximumNumPoints',10);
    h7 = animatedline('Color','b','MaximumNumPoints',10);
    h8 = animatedline('Color','g','MaximumNumPoints',10);
    h9 = animatedline('Color','m','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5', ...
        'Drone 6','Drone 7' ,'Drone 8')
    shg;

    vidObj = VideoWriter('8_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));
        addpoints(g5,data(i,7),data(i,8));
        addpoints(g6,data(i,9),data(i,10));
        addpoints(g7,data(i,11),data(i,12));
        addpoints(g8,data(i,13),data(i,14));
        addpoints(g9,data(i,15),data(i,16));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));
        addpoints(h5,data(i,7),data(i,8));
        addpoints(h6,data(i,9),data(i,10));
        addpoints(h7,data(i,11),data(i,12));
        addpoints(h8,data(i,13),data(i,14));
        addpoints(h9,data(i,15),data(i,16));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
elseif N == 9
    
    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    g6 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    g7 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g8 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g9 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g10 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);
    h5 = animatedline('Color','k','MaximumNumPoints',10);
    h6 = animatedline('Color','r','MaximumNumPoints',10);
    h7 = animatedline('Color','b','MaximumNumPoints',10);
    h8 = animatedline('Color','g','MaximumNumPoints',10);
    h9 = animatedline('Color','m','MaximumNumPoints',10);
    h10 = animatedline('Color','g','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5', ...
        'Drone 6','Drone 7' ,'Drone 8','Drone 9')
    shg;

    vidObj = VideoWriter('9_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));
        addpoints(g5,data(i,7),data(i,8));
        addpoints(g6,data(i,9),data(i,10));
        addpoints(g7,data(i,11),data(i,12));
        addpoints(g8,data(i,13),data(i,14));
        addpoints(g9,data(i,15),data(i,16));
        addpoints(g10,data(i,17),data(i,18));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));
        addpoints(h5,data(i,7),data(i,8));
        addpoints(h6,data(i,9),data(i,10));
        addpoints(h7,data(i,11),data(i,12));
        addpoints(h8,data(i,13),data(i,14));
        addpoints(h9,data(i,15),data(i,16));
        addpoints(h10,data(i,17),data(i,18));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);

elseif N ==  10

    trajectories_plot = figure();
    g1 = animatedline('Color','r','MaximumNumPoints',1,'Marker','x');
    g2 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g3 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g4 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g5 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    g6 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');
    g7 = animatedline('Color','b','MaximumNumPoints',1,'Marker','o');
    g8 = animatedline('Color','g','MaximumNumPoints',1,'Marker','o');
    g9 = animatedline('Color','m','MaximumNumPoints',1,'Marker','o');
    g10 = animatedline('Color','k','MaximumNumPoints',1,'Marker','o');
    g11 = animatedline('Color','r','MaximumNumPoints',1,'Marker','o');

    % Make animated line shorter to make it easier to view.
    h1 = animatedline('Color','r','MaximumNumPoints',10);
    h2 = animatedline('Color','b','MaximumNumPoints',10);
    h3 = animatedline('Color','g','MaximumNumPoints',10);
    h4 = animatedline('Color','m','MaximumNumPoints',10);
    h5 = animatedline('Color','k','MaximumNumPoints',10);
    h6 = animatedline('Color','r','MaximumNumPoints',10);
    h7 = animatedline('Color','b','MaximumNumPoints',10);
    h8 = animatedline('Color','g','MaximumNumPoints',10);
    h9 = animatedline('Color','m','MaximumNumPoints',10);
    h10 = animatedline('Color','g','MaximumNumPoints',10);
    h11 = animatedline('Color','m','MaximumNumPoints',10);

    axis([-ax_size,ax_size,-ax_size,ax_size]);
    legend('Target','Drone 1','Drone 2','Drone 3','Drone 4','Drone 5', ...
        'Drone 6','Drone 7' ,'Drone 8','Drone 9','Drone 10')
    shg;

    vidObj = VideoWriter('10_drones.avi');
    vidObj.FrameRate = 60;
    vidObj.Quality = 100;
    open(vidObj);

    for i = 1:5000

        % Update the drone positions.
        addpoints(g1,0,0);
        addpoints(g2,data(i,1),data(i,2));
        addpoints(g3,data(i,3),data(i,4));
        addpoints(g4,data(i,5),data(i,6));
        addpoints(g5,data(i,7),data(i,8));
        addpoints(g6,data(i,9),data(i,10));
        addpoints(g7,data(i,11),data(i,12));
        addpoints(g8,data(i,13),data(i,14));
        addpoints(g9,data(i,15),data(i,16));
        addpoints(g10,data(i,17),data(i,18));
        addpoints(g11,data(i,19),data(i,20));

        % Add new points to the trajectory lines.
        addpoints(h1,0,0);
        addpoints(h2,data(i,1),data(i,2));
        addpoints(h3,data(i,3),data(i,4));
        addpoints(h4,data(i,5),data(i,6));
        addpoints(h5,data(i,7),data(i,8));
        addpoints(h6,data(i,9),data(i,10));
        addpoints(h7,data(i,11),data(i,12));
        addpoints(h8,data(i,13),data(i,14));
        addpoints(h9,data(i,15),data(i,16));
        addpoints(h10,data(i,17),data(i,18));
        addpoints(h11,data(i,19),data(i,20));

        drawnow;
        currFrame = getframe;
        writeVideo(vidObj,currFrame);

    end

    close(vidObj);
    
end
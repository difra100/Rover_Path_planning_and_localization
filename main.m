%% ROVER FINAL EXERCISE (a.a. 2022-2023)

%%
clear
clc
load('exercise.mat')
init;
%% NOTE
% The 2D Cartesian landmarks coordinates are defined by the vectors
% (xLM, yLM). The two vectors Xvec and Yvec are used to georeference the
% maps (e.g., hazard map).

%%
%%% To compute the row and column indices (i,j) associated with a point of
%%% the map from its Cartesian coordinates (x0,y0), you can use the
%%% following piece of code
%% TASK 1
% Let us define a Via point to reach the P1 configuration, in order to
% avoid the shadowed regions.
% - Trajectory, Velocity, Theta, Theta_dot, Total_time

Pv1 = [-1.85*10^4; -7500];
Pv1 = [Pv1; atan2(-7500 + 2815, -1.85*10^4 + 23225)];

Pv2 = [-1.47*10^4; -9300];
Pv2 = [Pv2; atan2(-9300 + 7500, -1.47*10^4 + 1.85*10^4)];


P1 = [P1; atan2(P1(2)-Pv2(2), P1(1)-Pv2(1))];

limits = [Xvec; Yvec];
init_q = [P0(1:2)'];
init_vel = [];
init_theta = [P0(3)];
init_theta_d = [];
init_time = 0;

init_pixels = [get_pixels_coords(P0(1), P0(2))];

inter_pixels = [get_pixels_coords(P1(1), P1(2))]
goal_pixels = [get_pixels_coords(P1(1), P1(2))]
% goal_pixels(1) = goal_pixels(1)-100
% goal_pixels(2) = goal_pixels(2)+100
goal_pixels = [get_pixels_coords(Pf(1), Pf(2))]


path = A_star(inter_pixels', goal_pixels', obstacleMap);

plot_trajectory(obstacleMap, path, Xvec, Yvec)
% 
% [q_time1, velocity_t1, theta_t1, theta_d_t1, time1, pixels1] = Compute_trajectory(P0, Pv1, Kh, L, map, limits, init_q, init_vel, init_theta, init_theta_d, init_time, init_pixels);
% 
% [q_time2, velocity_t2, theta_t2, theta_d_t2, time2, pixels2] = Compute_trajectory(Pv1, Pv2, Kh, L, map, limits, q_time1, velocity_t1, theta_t1, theta_d_t1, time1, pixels1);
% 
% [q_time, velocity_t, theta_t, theta_d_t, time, pixels] = Compute_trajectory(Pv2, P1, Kh, L, map, limits, q_time2, velocity_t2, theta_t2, theta_d_t2, time2, pixels2);
% 

% figure
% plot((0:1:size(q_time,1)),q_time(:, 1),'r',(0:1:size(q_time,1)),q_time(:, 2))
% title('Positions')
% 
% figure
% plot((0:1:size(velocity_t,1))-1,velocity_t, 'r')
% title('Driving velocity')
% 
% figure
% plot((0:1:size(theta_t,1)-1),theta_t, 'r', (0:1:size(theta_d_t,1)-1),theta_d_t)
% title('Heading angle vs heading angle velocity')
% 
% disp('Time was: \n')
% disp(time)
% %
% %% Map of the environment
figure()
imshow(map,'XData',Xvec,'YData',Yvec);
set(gca,'Ydir','normal')
axis on
grid on
xlabel('X')
ylabel('Y')
hold on 
plot(path(:,1), path(:,2), 'yo','markersize',5,'linewidth',2)
% 
% 
% %
% %% Obstacle map
% %% NOTE: this is the map to be used for the path planning task by using
% %%       the A* algorithm
% figure();
% imshow(map,'XData',Xvec,'YData',Yvec);
% hold on
% set(gca,'Ydir','normal')
% 
% % 
% % %%
% %%% Landmarks
% figure()
% imshow(map,'XData',Xvec,'YData',Yvec);
% set(gca,'Ydir','normal')
% hold on
% plot(xLM, yLM, 'yo','markersize',5,'linewidth',2)

% 
% %%
% %%% For the Optional Task
% % figure()
% % imshow(map,'XData',Xvec,'YData',Yvec)
% % hold on
% % image(demCrop, 'AlphaData', 0.6, 'CDataMapping','scaled','XData',Xvec,'YData',Yvec);
% % colormap('parula');
% % set(gca,'Ydir','normal')
% % colorbar
% % axis on
% 
% % NOTE: the values reported in the colorbar are in the unit of meters.


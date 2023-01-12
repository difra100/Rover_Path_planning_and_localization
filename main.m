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
init_q =  zeros(60000000, 2); %[P0(1:2)'];

init_theta = zeros(60000000, 1);      %[P0(3)];
init_theta_d = zeros(60000000, 1);

init_vel = zeros(60000000, 1);
init_time = 0;

covar_det = zeros(60000000,1);


% init_pixels = [get_pixels_coords(P0(1), P0(2))];

inter_pixels = [get_pixels_coords(P1(1), P1(2))]
PO = [-1.8; 1.65]*10^4;
obs_pixels = [get_pixels_coords(PO(1), PO(2))]
% goal_pixels = [get_pixels_coords(P1(1), P1(2))]
% goal_pixels(1) = goal_pixels(1)-10
% goal_pixels(2) = goal_pixels(2)+10
goal_pixels = [get_pixels_coords(Pf(1), Pf(2))]
% % 
% % 



%% TASK 1: from P0 to P1
% 
% [q_time1, velocity_t1, theta_t1, theta_d_t1, time1, num1] = Compute_trajectory(P0, Pv1, Kh, L, map, limits, init_q, init_vel, init_theta, init_theta_d, init_time, 1, freq);
% 
% [q_time2, velocity_t2, theta_t2, theta_d_t2, time2, num2] = Compute_trajectory(Pv1, Pv2, Kh, L, map, limits, q_time1, velocity_t1, theta_t1, theta_d_t1, time1, num1, freq);
% 
% [q_time, velocity_t, theta_t, theta_d_t, time, num3] = Compute_trajectory(Pv2, P1, Kh, L, map, limits, q_time2, velocity_t2, theta_t2, theta_d_t2, time2, num2, freq);
% 

%% TASK 2: from P1 to Pf

slope = alpha;  % or zero

% path = A_star(inter_pixels', goal_pixels', obstacleMap, limits, demCrop, slope);
% 
% plot_trajectory(map, path, limits)

%% TASK 3: Localization with only Odometric measures, and with the help of the kalman filter.
% 
qs = load('-mat', './task1_vector/P0_P1_confs.dat');

thetas = load('-mat', './task1_vector/P0_P1_theta.dat');
% 

numbers = load('-mat', './task1_vector/P0_P1_numbers.dat');
numbers = numbers.numbers;


% init_vel = velocities.v(1:numbers(1),:);
% 
% velocity_t1 = velocities.v(numbers(1)+1:numbers(2),:);
% 
% velocity_t2 = velocities.v(numbers(2)+1:numbers(3), :);
% 
% init_theta_d = theta_d.td(1:numbers(1),:);
% 
% theta_d_t1 = theta_d.td(numbers(1):numbers(2),:);
% 
% theta_d_t2 = theta_d.td(numbers(2):numbers(3)-1, :);

old_q = qs.q(1:numbers(1),:);

old_q2 = qs.q(numbers(1)+1:numbers(2),:);

old_q3 = qs.q(numbers(2)+1:numbers(3), :);

old_theta = thetas.t(1:numbers(1),:);

old_theta2 = thetas.t(numbers(1)+1:numbers(2),:);

old_theta3 = thetas.t(numbers(2)+1:numbers(3), :);


kalman = 0;  % Employ the kalman engine
P_cell = cell(1, 6000000);
P_cell{1} = covariance_init;
[q_time1, theta_t1, time1, P_current1, covar_det1, P_cell, counter] = Localization_trajectory(P0, Pv1, covariance_init, P_cell, covar_det, od_noise_matrix, map, limits, init_q, old_q, init_theta, old_theta, init_time, xLM, yLM, instr_noise_var_matrix, instr_noise, maximum_dist, kalman, 1);
% 
[q_time2, theta_t2, time2, P_current2, covar_det2, P_cell, counter] = Localization_trajectory([q_time1(counter,1); q_time1(counter,2);theta_t1(counter,:)], Pv2, P_current1, P_cell, covar_det1, od_noise_matrix, map, limits, q_time1, old_q2, theta_t1, old_theta2, time1, xLM, yLM, instr_noise_var_matrix, instr_noise, maximum_dist, kalman, counter);
% 
[q_time, theta_t, time, P_current3, covar_det, P_cell, counter] = Localization_trajectory([q_time2(counter,1); q_time2(counter,2); theta_t2(counter,:)], P1, P_current2, P_cell, covar_det2, od_noise_matrix, map, limits, q_time2, old_q3, theta_t2, old_theta3, time2, xLM, yLM, instr_noise_var_matrix, instr_noise, maximum_dist, kalman, counter);
% 

plot_trajectory(map, qs.q, limits, 'm')
hold on
plot_trajectory(map, q_time(1:counter,:), limits, 'c')
hold on
for i = 1:2000:counter
    error_ellipse(P_cell{i}(1:2, 1:2), q_time(i,:)');
    hold on
end


%% Map of the environment
% figure()
% imshow(map,'XData',Xvec,'YData',Yvec);
% set(gca,'Ydir','normal')
% axis on
% grid on
% xlabel('X')
% ylabel('Y')
% hold on 
% % plot(path(:,1), path(:,2), 'yo','markersize',5,'linewidth',2)
% 
% 
% %
% %% Obstacle map
% %% NOTE: this is the map to be used for the path planning task by using
% %%       the A* algorithm
% figure();
% imshow(obstacleMap,'XData',Xvec,'YData',Yvec);
% hold on
% set(gca,'Ydir','normal')
% axis on
% grid on
% % 
% % %%
% %%% Landmarks
% figure()
% imshow(map,'XData',Xvec,'YData',Yvec);
% set(gca,'Ydir','normal')
% hold on
% plot(xLM, yLM, 'yo','markersize',5,'linewidth',2)
% 
% 
% %%
% %%% For the Optional Task
% figure()
% imshow(map,'XData',Xvec,'YData',Yvec)
% hold on
% image(demCrop, 'AlphaData', 0.6, 'CDataMapping','scaled','XData',Xvec,'YData',Yvec);
% colormap('parula');
% set(gca,'Ydir','normal')
% colorbar
% axis on

% NOTE: the values reported in the colorbar are in the unit of meters.
% 

%% ROVER FINAL EXERCISE (a.a. 2022-2023)


clear
clc
load('exercise.mat')
init;


task = 1 % Choose 1, 2 or 3, according to the defined enumeration.


%% TASK 1
% Let us define a Via point to reach the P1 configuration, in order to
% avoid the shadowed regions.
% - Trajectory, Velocity, Theta, Theta_dot, Total_time

Pv1 = [-1.7*10^4; -10000];
Pv1 = [Pv1; atan2(-10000 + 2815, -1.7*10^4 + 23225)];

Pv2 = [-0.7*10^4; -10000];
Pv2 = [Pv2; atan2(-10000 + 10000, -0.7*10^4 + 1.7*10^4)];

% Old via points
% Pv1 = [-18500; -7500];
% Pv1 = [Pv1; atan2(Pv1(2)-P0(2), Pv1(1)-P0(1))];
% 
% Pv2 = [-14700; -9300];
% Pv2 = [Pv2; atan2(Pv2(2)-Pv1(2), Pv2(1)-Pv1(1))];
%
P1 = [P1; atan2(P1(2)-Pv2(2), P1(1)-Pv2(1))];

limits = [Xvec; Yvec];

init_q =  zeros(60000000, 2);
    
init_theta = zeros(60000000, 1);      
init_theta_d = zeros(60000000, 1);

init_vel = zeros(60000000, 1);
init_time = 0;

init_pixels = [get_pixels_coords(P0(1), P0(2))];

inter_pixels = [get_pixels_coords(P1(1), P1(2))];
PO = [-1.8; 1.65]*10^4;
obs_pixels = [get_pixels_coords(PO(1), PO(2))]

goal_pixels = [get_pixels_coords(Pf(1), Pf(2))]

    
%% Pre-Allocation of important variables
if task == 1
    disp(' Task 1 started... ')
       
    
    %% TASK 1: from P0 to P1
    % Computations of the trajectory quantities, passing per every via point %
    
    [q_time1, velocity_t1, theta_t1, theta_d_t1, time1, num1] = Compute_trajectory(P0, Pv1, Kh, L, map, limits, init_q, init_vel, init_theta, init_theta_d, init_time, 1, freq);
    
    [q_time2, velocity_t2, theta_t2, theta_d_t2, time2, num2] = Compute_trajectory(Pv1, Pv2, Kh, L, map, limits, q_time1, velocity_t1, theta_t1, theta_d_t1, time1, num1, freq);
    
    [q_time, velocity_t, theta_t, theta_d_t, time, num3] = Compute_trajectory(Pv2, P1, Kh, L, map, limits, q_time2, velocity_t2, theta_t2, theta_d_t2, time2, num2, freq);
end

%% TASK 2: from P1 to Pf
if task == 2
    disp(' Task 2 started... ')
    slope = 0;  % 0: mandatory task, alpha: optional task
    limited_memory = false; % true if apply memory limitations.
    
    [path_planned, coords] = A_star(inter_pixels', goal_pixels', obstacleMap, limits, demCrop, slope, limited_memory);
    
    if path_planned ~= 0
        plot_trajectory(map, path_planned, limits, 'm')
    end
end

%% TASK 3: Localization with only Odometric measures, and with the help of the kalman filter.

if task == 3
    disp(' Task 3 started... ')
    qs = load('-mat', './task1_vector/P0_P1_confs.dat');  % Pre-computed odometry data
    
    thetas = load('-mat', './task1_vector/P0_P1_theta.dat'); % Pre-computed odometry data
 
    
    numbers = load('-mat', './task1_vector/P0_P1_numbers.dat'); % Via points gaps.
    numbers = numbers.numbers;
    covar_det = zeros(60000000,1);
    
    kalman = 0  % 0: only odometry, 1: Employ the kalman engine
    
    P_cell = cell(1, 6000000);
    P_cell{1} = covariance_init;
    
    [q_time1, theta_t1, time1, P_current1, covar_det1, P_cell, counter] = Localization_trajectory(P0, Pv1, covariance_init, P_cell, covar_det, od_noise_matrix, map, limits, init_q, qs.q, init_theta, thetas.t, init_time, xLM, yLM, instr_noise_var_matrix, instr_noise, maximum_dist, kalman, 1, numbers(1));
    
    [q_time2, theta_t2, time2, P_current2, covar_det2, P_cell, counter] = Localization_trajectory([q_time1(counter,1); q_time1(counter,2);theta_t1(counter,:)], Pv2, P_current1, P_cell, covar_det1, od_noise_matrix, map, limits, q_time1, qs.q, theta_t1, thetas.t, time1, xLM, yLM, instr_noise_var_matrix, instr_noise, maximum_dist, kalman, counter, numbers(2));
    
    [q_time, theta_t, time, P_current3, covar_det, P_cell, counter] = Localization_trajectory([q_time2(counter,1); q_time2(counter,2); theta_t2(counter,:)], P1, P_current2, P_cell, covar_det2, od_noise_matrix, map, limits, q_time2, qs.q, theta_t2, thetas.t, time2, xLM, yLM, instr_noise_var_matrix, instr_noise, maximum_dist, kalman, counter, numbers(3));
    
    
    plot_trajectory(obstacleMap, qs.q, limits, 'm') % Desired trajectory
    hold on
    plot_trajectory(obstacleMap, q_time(1:counter,:), limits, 'c')
    hold on
    for i = 1:2000:counter
        error_ellipse(P_cell{i}(1:2, 1:2), q_time(i,:)');  % This function plots the error ellipses.
        hold on
    end
    hold on
    plot(xLM, yLM, 'yo','markersize',5,'linewidth',2) % Look at the landmarks along with the result
end

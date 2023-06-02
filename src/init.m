Vmax = 20; % Maximum velocity (cm/s)
P0 = [-23225; -2815; -pi/4];
P1 = [-4855; -1975];
Pf = [- 18660; 29200];


%% Task 1 variables:
syms sv real
Kv = sv;
Kh = 0.1;
L = 1.5; % Axles distance

%% Task2 variables
alpha = 15*pi/180; % Maximum traversable slope

%% Task 3 variables:

freq = 1;
sampling_time = 1/freq;
kalman = 1    % 0: only odometry, 1: Employ the kalman engine
% ODOMETRIC noise
sigma_d = 0.005; % [m] noise on the travelled distance
sigma_theta = (0.05*pi)/180; % [rad] noise on the heading angle
od_noise_matrix = [sigma_d^2 0;
                0 sigma_theta^2];

% co-variance matrix diagonal values
sigma_x = 0.1;
sigma_y = sigma_x;
sigma_theta_2 = (1*pi)/180;

covariance_init = [sigma_x^2 0 0;
                   0 sigma_y^2 0; 
                   0 0 sigma_theta_2^2]; 

% LIDAR noise
sigma_r = 0.05; % [m] noise on lidar range measurement
sigma_beta = (0.3*pi)/180; % [rad] noise on lidar bearing angle

instr_noise = [sigma_r; sigma_beta];

instr_noise_var_matrix = [sigma_r^2 0;
                            0     sigma_beta^2];

maximum_dist = 250; % [m] maximum distance for the field of view.













%% ========================================================================
%  TurtleBot Navigation - Task 1, 2 & 3 Initialization
%  Ali Shavakhi - University of Padua (DEI)
% =========================================================================
clear all; close all; clc;

% [MODE SELECTION]
% 0 = Offline (Simulation) | 1 = Online (Real Robot)
is_real_robot = 0; 

% 1. MAP & TASKS
% Coordinates based on the laboratory map
x_coords = [-2.90, -1.60, -1.50, -1.40, -0.60, 0.20, 1.00, 1.45, 1.80, 2.20, 2.90];
y_coords = [-1.50, -1.10, -0.70, -0.30, -0.20, 0.10, 0.20, 1.50];

q_goal_task1 = [-1.60; -1.10; 0];
q_goal_task2 = [1.00; -0.30; 0];
dist_threshold = 0.05; % [m] position tolerance used by the task manager

% 2. ROBOT & SAMPLING
Ts = 0.04;    % Sampling Time
T_s = Ts;
r  = 0.03;    % Wheel Radius
d  = 0.165;   % Wheelbase

% 3. GAINS & SAFETY (Dynamic based on Mode)
if is_real_robot == 1
    % --- Real Robot Settings (Safe & Steady) ---
    k_x = 0.5; 
    k_y = 1.2; 
    k_theta = 0.3;

    v_max = 0.22; % [m/s] linear velocity limit, real-robot mode
    omega_max = 0.6; % [rad/s] body angular velocity limit, real-robot mode
    q_init = [-2.5; 1.0; -pi/2]; % initial pose inside the initial area (before Vicon lock)
    disp('>>> MODE: REAL ROBOT (Waiting for Vicon sync...)');
else
    % --- Simulation Settings (Fast & Sharp) ---
    k_x = 1.5; 
    k_y = 1.2; 
    k_theta = 0.3;

    v_max = 0.5; 
    omega_max = 1.5;
% [comment removed: the original Persian text was stored as 0x1A control bytes in the source file]
    q_init = [-2.9 + (1.4)*rand(); 0.1 + (1.4)*rand(); -pi/2 + (0.1*rand()-0.05)];
    disp('>>> MODE: SIMULATION (Random Pose Generated)');
end

disp('Workspace ready. Run Simulink now!');

%% CONTROLLER SELECTION
controller_index = 2;   % 1 linear, 2 nonlinear, 3 feedback linearization

%% CONTROLLER PARAMETERS
if controller_index == 1

    xi = 0.85;
    a = 6;
    control_par = [xi, a, 0];

elseif controller_index == 2

    xi = 0.85;
    b = 5;
    control_par = [xi, b, 0];

else

    k1 = 2.0;
    k2 = 2.0;
    b = 0.08;
    control_par = [k1, k2, b];

end
%% EKF
% prob_gps_loss = 0.9;
% EKF initial covariance
P_INIT_EKF = diag([0.001, 0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6*Ts, 0.0175/6*Ts].^2);
ENCODER_QUANTIZATION = 0.0175;

% EKF process noice covariance
D = diag([0.001, 0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6*Ts, 0.0175/6*Ts].^2);

% EKF measurement noise
R_2 = diag([ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6].^2);                         %(delta phi wheels angles)

R_5 = diag(([0.001, 0.001, 0.001, ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6]).^2);  %
% Replace sigma_noise with the actual standard deviation from your Simulink block
sigma_noise = 0.01; 

R_bad = diag(([sigma_noise, sigma_noise, sigma_noise, ENCODER_QUANTIZATION/6, ENCODER_QUANTIZATION/6]).^2);
R_good  = R_5;
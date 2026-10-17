clear; clc; close all;

% ==============================
% Simulation from real Task 1 initial condition
% ==============================

is_real_robot = 0;   % simulation mode

% Map coordinates
x_coords = [-2.90, -1.60, -1.50, -1.40, -0.60, 0.20, 1.00, 1.45, 1.80, 2.20, 2.90];
y_coords = [-1.50, -1.10, -0.70, -0.30, -0.20, 0.10, 0.20, 1.50];

% Task goals
q_goal_task1 = [-1.60; -1.10; 0];
q_goal_task2 = [1.00; -0.30; 0];

dist_threshold = 0.05;

% Robot parameters
Ts = 0.04;
r  = 0.03;
d  = 0.165;

% Same gains used in real robot test
k_x = 0.5;
k_y = 1.2;
k_theta = 0.4;

% Use same safe limits as real experiment for fair comparison
v_max = 0.22;
omega_max = 0.6;

% Initial condition taken from real Task 1 after localization transient
q_init = [-2.5565; 0.5193; -1.5281];

disp('Simulation initialized from real Task 1 initial condition.');
disp('q_init = ');
disp(q_init);
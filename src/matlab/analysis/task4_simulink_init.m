%% task4_simulink_init.m
% Initialization file for Task 4 Simulink simulation
%
% Purpose:
% - Separate the reference initial pose, true robot initial pose,
%   and EKF initial estimate.
% - Enable a wrong-EKF-initialization test:
%
%       qhat0 ~= q_true0
%
% Important:
% - q_true0 must initialize the simulated robot / unicycle plant.
% - qhat0 must initialize the LiDAR-EKF block.
% - q_ref0 must remain the nominal reference start.

clearvars -except out
clc;

%% ============================================================
%  Simulation and robot parameters
% ============================================================

Ts = 0.04;              % sampling time [s]

r = 0.03;               % wheel radius [m]
d = 0.165;              % wheel distance [m]
wM = 10;                % wheel speed saturation [rad/s]

Tstop = 45.4159;        % simulation stop time [s]

%% ============================================================
%  Tracking controller gains
% ============================================================

k_x = 0.75;
k_y = 2.00;
k_theta = 1.20;

%% ============================================================
%  Select Task 4 simulation case
% ============================================================
% Available modes:
%
% 1) "exact"
%    q_true0 = q_ref0
%    qhat0   = q_true0
%
% 2) "wrong_ekf_initialization"
%    q_true0 = q_ref0
%    qhat0   = q_true0 + delta_q0
%
% 3) "reference_mismatch_old"
%    q_true0 = q_ref0 + delta_q0
%    qhat0   = q_true0
%
% For the new required test, use:
% simCase = "wrong_ekf_initialization";

simCase = "exact";

%% ============================================================
%  Initial poses
% ============================================================
% q_ref0:
% Nominal initial pose used by the Task 4 reference trajectory.
%
% IMPORTANT:
% If your final Task 4 reference manager uses a different start,
% replace this value with the exact same value used there.

q_ref0 = [0.960; -0.340; deg2rad(4.00)];

% Perturbation used for non-exact initialization tests.
% This is an artificial simulation perturbation, not a real sensor value.

delta_q0 = [0.05; -0.05; deg2rad(5.00)];

switch simCase

    case "exact"

        % Exact initialization:
        % true robot, reference, and EKF start consistently.
        q_true0 = q_ref0;
        qhat0   = q_true0;

        P0_ekf = diag([1e-4, 1e-4, deg2rad(1)^2]);

        lidar_gate_initial = 0.045;
        lidar_gate_after   = 0.045;
        min_valid_lidar    = 4;
        R_lidar_scalar     = (0.010)^2;

    case "wrong_ekf_initialization"

        % Correct test for localization-initialization uncertainty:
        % the actual robot starts from q_true0, but the EKF starts
        % from a wrong pose qhat0.
        %
        % This produces nonzero initial localization error.

        q_true0 = q_ref0;

        qhat0 = q_true0 + delta_q0;
        qhat0(3) = wrapToPiLocal(qhat0(3));

        % Larger covariance because the EKF initial pose is uncertain.
        P0_ekf = diag([5e-3, 5e-3, deg2rad(7)^2]);

        % Looser initial LiDAR gate to allow recovery from wrong qhat0.
        lidar_gate_initial = 0.120;
        lidar_gate_after   = 0.080;
        min_valid_lidar    = 3;
        R_lidar_scalar     = (0.006)^2;

    case "reference_mismatch_old"

        % Old non-exact case:
        % the robot starts from a perturbed pose with respect to the
        % reference, but the EKF is initialized consistently with q_true0.
        %
        % This gives initial tracking error, but zero initial localization error.

        q_true0 = q_ref0 + delta_q0;
        q_true0(3) = wrapToPiLocal(q_true0(3));

        qhat0 = q_true0;

        P0_ekf = diag([5e-3, 5e-3, deg2rad(7)^2]);

        lidar_gate_initial = 0.120;
        lidar_gate_after   = 0.080;
        min_valid_lidar    = 3;
        R_lidar_scalar     = (0.006)^2;

    otherwise
        error("Unknown simCase: %s", simCase);

end

%% ============================================================
%  Compatibility variables for current Simulink model
% ============================================================
% These are kept because your current Simulink model may still use
% old variable names such as q0_true or q_init.
%
% Later, in Simulink, we will connect:
%   q_true0 --> Unicycle / simulated robot initial condition
%   qhat0   --> LiDAR-EKF initial condition

q0_true = q_true0;      % for simulated robot initial condition
q_init  = qhat0;        % for EKF initial condition, if current port is named q_init

% If your LiDAR_EKF block uses P0 directly, use this variable.
P0 = P0_ekf;

%% ============================================================
%  Artificial sensor perturbations
% ============================================================
% These values are simulation assumptions only.
% They are not professor-provided sensor specifications.

sigma_lidar     = 0.006;    % LiDAR range noise standard deviation [m]
p_lidar_dropout = 0.03;     % LiDAR dropout probability [-]
sigma_encoder   = 0.0008;   % encoder increment noise standard deviation [rad]
sigma_imu_w     = 0.010;    % IMU yaw-rate noise standard deviation [rad/s]

%% ============================================================
%  EKF process covariance
% ============================================================

Q_ekf = diag([1.5e-5, 1.5e-5, 6e-5]);

%% ============================================================
%  LiDAR geometry
% ============================================================

lidar_offset = 0.076;       % LiDAR offset along sagittal axis [m]

N_lidar = 36;
theta_lidar = zeros(N_lidar,1);

for i = 1:N_lidar
    theta_lidar(i) = -pi + (i-1)*(2*pi/N_lidar);
end

%% ============================================================
%  Cube geometry
% ============================================================
% Known cube used by the LiDAR ray-intersection model.
% Current Task 4 plots show cube centered at [2.0, 0.0] with side 0.4 m.

cube_center = [2.0; 0.0];
cube_side   = 0.4;

cube_x_min = cube_center(1) - cube_side/2;
cube_x_max = cube_center(1) + cube_side/2;
cube_y_min = cube_center(2) - cube_side/2;
cube_y_max = cube_center(2) + cube_side/2;

%% ============================================================
%  Reset flags
% ============================================================
% Later in Simulink we should separate true-state reset and EKF reset.
% For now these variables are available in the workspace.

reset_task  = 0;
reset_true  = 0;
reset_ekf   = 0;
reset_noise = 0;

%% ============================================================
%  Numerical initial-error checks
% ============================================================

initial_tracking_pos_error = norm(q_true0(1:2) - q_ref0(1:2));
initial_tracking_heading_error_deg = rad2deg(wrapToPiLocal(q_ref0(3) - q_true0(3)));

initial_localization_pos_error = norm(qhat0(1:2) - q_true0(1:2));
initial_localization_heading_error_deg = rad2deg(wrapToPiLocal(qhat0(3) - q_true0(3)));

fprintf('\n============================================================\n');
fprintf('Task 4 initialization loaded\n');
fprintf('Simulation case: %s\n', simCase);
fprintf('============================================================\n');

fprintf('q_ref0   = [%.4f, %.4f, %.2f deg]\n', ...
    q_ref0(1), q_ref0(2), rad2deg(q_ref0(3)));

fprintf('q_true0  = [%.4f, %.4f, %.2f deg]\n', ...
    q_true0(1), q_true0(2), rad2deg(q_true0(3)));

fprintf('qhat0    = [%.4f, %.4f, %.2f deg]\n', ...
    qhat0(1), qhat0(2), rad2deg(qhat0(3)));

fprintf('\nInitial tracking position error      = %.4f m\n', ...
    initial_tracking_pos_error);
fprintf('Initial tracking heading error       = %.2f deg\n', ...
    initial_tracking_heading_error_deg);

fprintf('Initial localization position error  = %.4f m\n', ...
    initial_localization_pos_error);
fprintf('Initial localization heading error   = %.2f deg\n', ...
    initial_localization_heading_error_deg);

fprintf('\nEKF P0 = diag([%.4g, %.4g, %.4g])\n', ...
    P0_ekf(1,1), P0_ekf(2,2), P0_ekf(3,3));

fprintf('LiDAR gate initial / after = %.3f / %.3f m\n', ...
    lidar_gate_initial, lidar_gate_after);
fprintf('Minimum valid LiDAR rays   = %d\n', min_valid_lidar);
fprintf('LiDAR R scalar             = %.6g\n', R_lidar_scalar);

fprintf('============================================================\n\n');

%% ============================================================
%  Local helper function
% ============================================================

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end
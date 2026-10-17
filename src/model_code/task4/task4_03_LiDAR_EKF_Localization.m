% Extracted verbatim from task4_simulation.slx (simulink/stateflow/chart_51.xml) by tools/extract_model_code.py
function [q_est, innovation_RMS, n_valid_lidar] = LiDAR_EKF_Localization(v_odo, omega_odo, z_lidar, Ts, q_init, reset_ekf)
%#codegen
% LiDAR-based EKF localization for Task 4
%
% Prediction:
%   Odometry / IMU-based unicycle prediction
%
% Correction:
%   Nonlinear LiDAR range model to the known cube walls
%
% This version supports both:
%   1) exact initial point
%   2) non-exact initial point
%
% The EKF automatically selects the initial covariance and LiDAR update
% parameters according to q_init.

persistent q_hat P initialized init_case k_step

%% ============================================================
% 1. Initialization
% ============================================================

if isempty(initialized) || reset_ekf > 0.5

    q_hat = q_init;
    q_hat(3) = wrapToPiLocal(q_hat(3));

    % Nominal initial pose used by the reference trajectory
    q0_nominal_ref = [0.96; -0.34; 4*pi/180];

    init_pos_error = sqrt((q_init(1) - q0_nominal_ref(1))^2 + ...
                          (q_init(2) - q0_nominal_ref(2))^2);

    init_ang_error = abs(wrapToPiLocal(q_init(3) - q0_nominal_ref(3)));

    % Automatically select initialization mode.
    % init_case = 0: exact initial point
    % init_case = 1: non-exact initial point
    if init_pos_error < 1e-5 && init_ang_error < 1e-5

        init_case = 0;

        % Exact initialization:
        % small initial covariance because the initial pose is trusted.
        P = diag([1e-4, 1e-4, (1*pi/180)^2]);

    else

        init_case = 1;

        % Non-exact initialization:
        % larger initial covariance because the initial pose is uncertain
        % with respect to the nominal reference.
        P = diag([5e-3, 5e-3, (7*pi/180)^2]);

    end

    k_step = 0;
    initialized = true;

end

%% ============================================================
% 2. EKF parameters
% ============================================================

% Process noise covariance
Q = diag([1.5e-5, 1.5e-5, 6e-5]);

% Step counter for adaptive gating
k_step = k_step + 1;

% Adaptive LiDAR update parameters:
%
% Exact initial point:
%   The initial pose is trusted. Therefore, the EKF should not be moved
%   too aggressively by noisy LiDAR rays. We use stricter gating, require
%   more valid rays, and use a larger LiDAR measurement covariance.
%
% Non-exact initial point:
%   The initial pose is intentionally perturbed with respect to the nominal
%   reference. Therefore, the filter is allowed to accept a wider gate at
%   the beginning, so that the LiDAR update can correct the transient.
if init_case == 0

    % Exact initial point
    gate = 0.045;
    min_valid = 4;
    R_scalar = (0.010)^2;

else

    % Non-exact initial point
    if k_step <= 60       % about 2.4 s for Ts = 0.04 s
        gate = 0.120;
    else
        gate = 0.080;
    end

    min_valid = 3;
    R_scalar = (0.006)^2;

end

%% ============================================================
% 3. Prediction step
% ============================================================

theta = q_hat(3);
theta_mid = theta + 0.5*Ts*omega_odo;

q_pred = zeros(3,1);
q_pred(1) = q_hat(1) + Ts*v_odo*cos(theta_mid);
q_pred(2) = q_hat(2) + Ts*v_odo*sin(theta_mid);
q_pred(3) = wrapToPiLocal(q_hat(3) + Ts*omega_odo);

% Jacobian of prediction model
F = eye(3);
F(1,3) = -Ts*v_odo*sin(theta_mid);
F(2,3) =  Ts*v_odo*cos(theta_mid);

P_pred = F*P*F' + Q;
P_pred = 0.5*(P_pred + P_pred');

%% ============================================================
% 4. LiDAR prediction and numerical Jacobian
% ============================================================

h_pred = lidarModel(q_pred);
H_full = numericalLidarJacobian(q_pred);

%% ============================================================
% 5. Count valid gated LiDAR rays
% ============================================================

n_gate = 0;

for i = 1:36

    if isfinite(z_lidar(i)) && isfinite(h_pred(i)) && ...
       isfinite(H_full(i,1)) && isfinite(H_full(i,2)) && isfinite(H_full(i,3))

        innov_i = z_lidar(i) - h_pred(i);

        if abs(innov_i) < gate
            n_gate = n_gate + 1;
        end

    end

end

%% ============================================================
% 6. If not enough LiDAR rays are available, skip update
% ============================================================

if n_gate < min_valid

    q_hat = q_pred;
    P = P_pred;

    q_est = q_hat;
    innovation_RMS = NaN;
    n_valid_lidar = n_gate;

    return;

end

%% ============================================================
% 7. Sequential EKF update with scalar LiDAR measurements
% ============================================================

q_upd = q_pred;
P_upd = P_pred;

sum_innov2 = 0.0;
n_used = 0;

for i = 1:36

    if isfinite(z_lidar(i)) && isfinite(h_pred(i)) && ...
       isfinite(H_full(i,1)) && isfinite(H_full(i,2)) && isfinite(H_full(i,3))

        innov_i = z_lidar(i) - h_pred(i);

        if abs(innov_i) < gate

            H = H_full(i,:);

            S = H*P_upd*H' + R_scalar;

            if S > 1e-12

                K = (P_upd*H')/S;

                q_upd = q_upd + K*innov_i;
                q_upd(3) = wrapToPiLocal(q_upd(3));

                I = eye(3);

                % Joseph-form covariance update for numerical stability
                P_upd = (I - K*H)*P_upd*(I - K*H)' + K*R_scalar*K';
                P_upd = 0.5*(P_upd + P_upd');

                sum_innov2 = sum_innov2 + innov_i^2;
                n_used = n_used + 1;

            end

        end

    end

end

%% ============================================================
% 8. Save updated state
% ============================================================

q_hat = q_upd;
P = P_upd;

q_est = q_hat;
n_valid_lidar = n_used;

if n_used > 0
    innovation_RMS = sqrt(sum_innov2/n_used);
else
    innovation_RMS = NaN;
end

end

%% ============================================================
% LiDAR measurement model
% ============================================================

function h = lidarModel(q)

lidar_offset = 0.076;

x = q(1);
y = q(2);
theta = q(3);

% LiDAR center translated along robot sagittal axis
pL = [x; y] + lidar_offset*[cos(theta); sin(theta)];

h = NaN(36,1);

for i = 1:36

    angle_rel = -pi + (i-1)*(2*pi/36);
    alpha = theta + angle_rel;

    h(i) = rayBoxDistance(pL, alpha);

end

end

%% ============================================================
% Numerical LiDAR Jacobian
% ============================================================

function H = numericalLidarJacobian(q)

epsVal = [1e-5; 1e-5; 1e-5];

H = NaN(36,3);

for j = 1:3

    qp = q;
    qm = q;

    qp(j) = qp(j) + epsVal(j);
    qm(j) = qm(j) - epsVal(j);

    qp(3) = wrapToPiLocal(qp(3));
    qm(3) = wrapToPiLocal(qm(3));

    hp = lidarModel(qp);
    hm = lidarModel(qm);

    for i = 1:36

        if isfinite(hp(i)) && isfinite(hm(i))
            H(i,j) = (hp(i) - hm(i))/(2*epsVal(j));
        else
            H(i,j) = NaN;
        end

    end

end

end

%% ============================================================
% Ray-box intersection model
% ============================================================

function dist = rayBoxDistance(p, alpha)

% Known cube geometry
cube_xmin = 1.80;
cube_xmax = 2.20;
cube_ymin = -0.20;
cube_ymax = 0.20;

lidar_max_range = 3.0;

dir = [cos(alpha); sin(alpha)];

best = 1e9;
found = false;

%% Intersections with vertical cube walls

if abs(dir(1)) > 1e-10

    tx1 = (cube_xmin - p(1))/dir(1);
    yx1 = p(2) + tx1*dir(2);

    if tx1 > 1e-6 && yx1 >= cube_ymin && yx1 <= cube_ymax
        best = min(best, tx1);
        found = true;
    end

    tx2 = (cube_xmax - p(1))/dir(1);
    yx2 = p(2) + tx2*dir(2);

    if tx2 > 1e-6 && yx2 >= cube_ymin && yx2 <= cube_ymax
        best = min(best, tx2);
        found = true;
    end

end

%% Intersections with horizontal cube walls

if abs(dir(2)) > 1e-10

    ty1 = (cube_ymin - p(2))/dir(2);
    xy1 = p(1) + ty1*dir(1);

    if ty1 > 1e-6 && xy1 >= cube_xmin && xy1 <= cube_xmax
        best = min(best, ty1);
        found = true;
    end

    ty2 = (cube_ymax - p(2))/dir(2);
    xy2 = p(1) + ty2*dir(1);

    if ty2 > 1e-6 && xy2 >= cube_xmin && xy2 <= cube_xmax
        best = min(best, ty2);
        found = true;
    end

end

%% Output distance

if found && best <= lidar_max_range
    dist = best;
else
    dist = NaN;
end

end

%% ============================================================
% Angle wrapping
% ============================================================

function a = wrapToPiLocal(a)

a = atan2(sin(a), cos(a));

end

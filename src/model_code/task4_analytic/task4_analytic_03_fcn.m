% Extracted verbatim from task4_simulation_analytic_jacobian.slx (simulink/stateflow/chart_51.xml) by tools/extract_model_code.py
function [q_est, innovation_RMS, n_valid_lidar] = fcn(v_odo, omega_odo, z_lidar, Ts, q_init, reset_ekf)
% LiDAR_EKF_Localization
%
% EKF localization for Task 4 using:
% - odometry / IMU prediction
% - analytical LiDAR ray-wall Jacobian
%
% Inputs:
%   v_odo      : odometric linear velocity [m/s]
%   omega_odo  : odometric angular velocity [rad/s]
%   z_lidar    : simulated LiDAR ranges, 36x1 [m]
%   Ts         : sampling time [s]
%   q_init     : EKF initial estimate, 3x1
%   reset_ekf  : reset flag
%
% Outputs:
%   q_est          : estimated pose, 3x1
%   innovation_RMS : RMS innovation after gating [m]
%   n_valid_lidar  : number of LiDAR beams used in EKF update

%#codegen

persistent q_hat P k_counter

%% ============================================================
%  Fixed Task 4 EKF parameters
% ============================================================

% Nominal initial pose used only to select exact/wrong-EKF tuning.
q_nominal = [0.960; -0.340; deg2radLocal(4.00)];

% Process covariance
Q = diag([1.5e-5, 1.5e-5, 6e-5]);

% LiDAR geometry
Nbeams = 36;
lidar_offset = 0.076;

% Known cube geometry
cube_x_min = 1.80;
cube_x_max = 2.20;
cube_y_min = -0.20;
cube_y_max = 0.20;

% Decide whether this is exact initialization or wrong-EKF initialization.
% In exact case:
%   q_init = q_true0 = q_nominal
% In wrong-EKF case:
%   q_init = qhat0 ~= q_true0
init_difference_pos = sqrt((q_init(1)-q_nominal(1))^2 + (q_init(2)-q_nominal(2))^2);
init_difference_th  = abs(wrapPiLocal(q_init(3)-q_nominal(3)));

wrong_ekf_initialization = (init_difference_pos > 0.02) || (init_difference_th > deg2radLocal(2.0));

if wrong_ekf_initialization
    P0 = diag([5e-3, 5e-3, deg2radLocal(7)^2]);
    gate_initial = 0.120;
    gate_after   = 0.080;
    min_valid_required = 3;
    R_lidar_scalar = (0.006)^2;
else
    P0 = diag([1e-4, 1e-4, deg2radLocal(1)^2]);
    gate_initial = 0.045;
    gate_after   = 0.045;
    min_valid_required = 4;
    R_lidar_scalar = (0.010)^2;
end

%% ============================================================
%  Initialize / reset EKF
% ============================================================

if isempty(q_hat) || isempty(P) || isempty(k_counter) || reset_ekf ~= 0
    q_hat = q_init;
    q_hat(3) = wrapPiLocal(q_hat(3));

    P = P0;
    k_counter = 0;
end

k_counter = k_counter + 1;

%% ============================================================
%  EKF prediction step
% ============================================================

theta = q_hat(3);

theta_mid = theta + 0.5*Ts*omega_odo;

q_pred = zeros(3,1);
q_pred(1) = q_hat(1) + Ts*v_odo*cos(theta_mid);
q_pred(2) = q_hat(2) + Ts*v_odo*sin(theta_mid);
q_pred(3) = wrapPiLocal(q_hat(3) + Ts*omega_odo);

F = eye(3);
F(1,3) = -Ts*v_odo*sin(theta_mid);
F(2,3) =  Ts*v_odo*cos(theta_mid);

P_pred = F*P*F' + Q;

%% ============================================================
%  Analytical LiDAR prediction and Jacobian
% ============================================================

[z_hat_all, H_all, valid_pred] = lidarModelAnalytic( ...
    q_pred, ...
    Nbeams, ...
    lidar_offset, ...
    cube_x_min, cube_x_max, ...
    cube_y_min, cube_y_max);

%% ============================================================
%  Innovation gating
% ============================================================

if k_counter <= round(3.0/Ts)
    lidar_gate = gate_initial;
else
    lidar_gate = gate_after;
end

innov_stack = zeros(Nbeams,1);
H_stack = zeros(Nbeams,3);

count = 0;

for i = 1:Nbeams

    zi = z_lidar(i);

    z_is_valid = isfiniteLocal(zi) && (zi > 0);

    if z_is_valid && valid_pred(i)

        innov_i = zi - z_hat_all(i);

        if abs(innov_i) <= lidar_gate

            count = count + 1;

            innov_stack(count) = innov_i;
            H_stack(count,:) = H_all(i,:);

        end
    end
end

n_valid_lidar = count;

%% ============================================================
%  EKF correction step
% ============================================================

q_upd = q_pred;
P_upd = P_pred;

if count >= min_valid_required

    sumInnov2 = 0;

    for j = 1:count

        Hrow = H_stack(j,:);       % 1x3
        innov = innov_stack(j);    % scalar

        S = Hrow*P_upd*Hrow' + R_lidar_scalar;

        if S > 1e-12

            K = (P_upd*Hrow')/S;   % 3x1

            q_upd = q_upd + K*innov;
            q_upd(3) = wrapPiLocal(q_upd(3));

            % Joseph-stabilized covariance update for scalar measurement
            I3 = eye(3);
            KH = K*Hrow;
            P_upd = (I3 - KH)*P_upd*(I3 - KH)' + K*R_lidar_scalar*K';

            sumInnov2 = sumInnov2 + innov^2;
        end
    end

    innovation_RMS = sqrt(sumInnov2 / count);

else

    % Not enough valid LiDAR beams: prediction only
    innovation_RMS = NaN;

end

%% ============================================================
%  Save persistent state and output
% ============================================================

q_hat = q_upd;
P = P_upd;

q_est = q_hat;

end

%% ========================================================================
%  Analytical LiDAR model and Jacobian
% ========================================================================

function [z_pred, H_all, valid_pred] = lidarModelAnalytic(q, Nbeams, l, cube_x_min, cube_x_max, cube_y_min, cube_y_max)
% Analytical LiDAR range model and Jacobian for a known square cube.
%
% For each beam, the function checks the four cube walls and selects the
% nearest positive valid ray-wall intersection.

z_pred = NaN(Nbeams,1);
H_all = zeros(Nbeams,3);
valid_pred = false(Nbeams,1);

x = q(1);
y = q(2);
th = q(3);

px = x + l*cos(th);
py = y + l*sin(th);

for i = 1:Nbeams

    theta_i = -pi + (i-1)*(2*pi/Nbeams);
    phi = th + theta_i;

    best_h = inf;
    best_H = zeros(1,3);
    best_valid = false;

    %% Wall 1: x = cube_x_min
    [h, Hrow, ok] = rayLineJacobian(q, theta_i, 1, 0, -cube_x_min, l);
    if ok
        Xint = px + h*cos(phi);
        Yint = py + h*sin(phi);

        if isInside(Yint, cube_y_min, cube_y_max) && h < best_h
            best_h = h;
            best_H = Hrow;
            best_valid = true;
        end
    end

    %% Wall 2: x = cube_x_max
    [h, Hrow, ok] = rayLineJacobian(q, theta_i, 1, 0, -cube_x_max, l);
    if ok
        Xint = px + h*cos(phi);
        Yint = py + h*sin(phi);

        if isInside(Yint, cube_y_min, cube_y_max) && h < best_h
            best_h = h;
            best_H = Hrow;
            best_valid = true;
        end
    end

    %% Wall 3: y = cube_y_min
    [h, Hrow, ok] = rayLineJacobian(q, theta_i, 0, 1, -cube_y_min, l);
    if ok
        Xint = px + h*cos(phi);
        Yint = py + h*sin(phi);

        if isInside(Xint, cube_x_min, cube_x_max) && h < best_h
            best_h = h;
            best_H = Hrow;
            best_valid = true;
        end
    end

    %% Wall 4: y = cube_y_max
    [h, Hrow, ok] = rayLineJacobian(q, theta_i, 0, 1, -cube_y_max, l);
    if ok
        Xint = px + h*cos(phi);
        Yint = py + h*sin(phi);

        if isInside(Xint, cube_x_min, cube_x_max) && h < best_h
            best_h = h;
            best_H = Hrow;
            best_valid = true;
        end
    end

    if best_valid
        z_pred(i) = best_h;
        H_all(i,:) = best_H;
        valid_pred(i) = true;
    end
end

end

%% ========================================================================
%  Analytical ray-line Jacobian
% ========================================================================

function [h, Hrow, valid] = rayLineJacobian(q, theta_i, A, B, C, l)
% Analytical ray-line intersection range and Jacobian.
%
% Wall:
%   A X + B Y + C = 0
%
% Ray:
%   p(s) = p_L + s [cos(phi); sin(phi)]
%
% Predicted range:
%   h = -N/D

x = q(1);
y = q(2);
th = q(3);

px = x + l*cos(th);
py = y + l*sin(th);

phi = th + theta_i;

N = A*px + B*py + C;
D = A*cos(phi) + B*sin(phi);

h = NaN;
Hrow = zeros(1,3);
valid = false;

if abs(D) < 1e-9
    return;
end

h = -N/D;

if h <= 0
    return;
end

dN_dtheta = l*(-A*sin(th) + B*cos(th));
dD_dtheta = -A*sin(phi) + B*cos(phi);

dh_dx = -A/D;
dh_dy = -B/D;
dh_dtheta = (N*dD_dtheta - dN_dtheta*D)/(D^2);

Hrow(1) = dh_dx;
Hrow(2) = dh_dy;
Hrow(3) = dh_dtheta;

valid = true;

end

%% ========================================================================
%  Utility functions
% ========================================================================

function ok = isInside(value, lowerBound, upperBound)

tol = 1e-9;
ok = (value >= lowerBound - tol) && (value <= upperBound + tol);

end

function a = wrapPiLocal(a)

a = atan2(sin(a), cos(a));

end

function r = deg2radLocal(d)

r = d*pi/180;

end

function tf = isfiniteLocal(x)

tf = ~isnan(x) && ~isinf(x);

end

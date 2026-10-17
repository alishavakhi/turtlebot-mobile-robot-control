% Extracted verbatim from task3_simulation.slx (simulink/stateflow/chart_121.xml) by tools/extract_model_code.py
function [q_target, v_ref, omega_ref, ctrl_mode, state_out, task3_active, task3_done, elapsed_circle] = Task_Manager(t, q_est, reset_task)
%#codegen
% Task 3 manager:
% State 1: move from Task 2 end to Task 3 circular start
% State 2: circular trajectory tracking
% State 3: stop / hold final pose

persistent state t_circle0 q_hold

if isempty(state) || reset_task > 0.5
    state = 1;
    t_circle0 = 0.0;
    q_hold = zeros(3,1);
end

%% ============================================================
% Task 3 geometry
% ============================================================

% Task 3 starting point: [x8, y4]
x_start = 1.45;
y_start = -0.30;

% Cube / circle center
xc = 2.00;
yc = 0.00;

dx0 = x_start - xc;
dy0 = y_start - yc;

R_circle = sqrt(dx0^2 + dy0^2);

% Counter-clockwise circle
circle_dir = 1.0;

% Circle angular speed
omega_circle_abs = 0.30;      % [rad/s]

% One and a half turns
N_turns = 1.5;
T_circle = 2*pi*N_turns/omega_circle_abs;

% Initial angular coordinate on circle
alpha0 = atan2(dy0, dx0);

% Tangent heading at the circle start
theta_start = wrapToPiLocal(alpha0 + circle_dir*pi/2);

q_start = zeros(3,1);
q_start(1) = x_start;
q_start(2) = y_start;
q_start(3) = theta_start;

%% ============================================================
% Switching thresholds
% ============================================================
% These numbers are chosen to avoid two bad cases:
% 1) starting the circle too early with bad heading;
% 2) stopping forever at the start point.

pos_thr_start   = 0.050;     % [m]
theta_thr_start = 0.05;      % [rad], about 20 deg

%% ============================================================
% Default outputs
% ============================================================

q_target = zeros(3,1);
q_target(1) = q_start(1);
q_target(2) = q_start(2);
q_target(3) = q_start(3);

v_ref = 0.0;
omega_ref = 0.0;
ctrl_mode = 1.0;
elapsed_circle = 0.0;

%% ============================================================
% State 1: go to circular start
% ============================================================

if state == 1

    q_target(1) = q_start(1);
    q_target(2) = q_start(2);
    q_target(3) = q_start(3);

    v_ref = 0.0;
    omega_ref = 0.0;
    ctrl_mode = 1.0;

    ex = q_start(1) - q_est(1);
    ey = q_start(2) - q_est(2);
    eth = wrapToPiLocal(q_start(3) - q_est(3));

    pos_err = sqrt(ex^2 + ey^2);
    theta_err = abs(eth);

    normal_switch = (pos_err < pos_thr_start) && (theta_err < theta_thr_start);

    if normal_switch
        state = 2;
        t_circle0 = t;
        elapsed_circle = 0.0;
    end
end

%% ============================================================
% State 2: circular tracking
% ============================================================

if state == 2

    elapsed_circle = t - t_circle0;

    if elapsed_circle < 0.0
        elapsed_circle = 0.0;
    end

    if elapsed_circle > T_circle
        elapsed_circle = T_circle;
    end

    alpha = alpha0 + circle_dir*omega_circle_abs*elapsed_circle;

    xd = xc + R_circle*cos(alpha);
    yd = yc + R_circle*sin(alpha);
    thd = wrapToPiLocal(alpha + circle_dir*pi/2);

    q_target(1) = xd;
    q_target(2) = yd;
    q_target(3) = thd;

    v_ref = R_circle*omega_circle_abs;
    omega_ref = circle_dir*omega_circle_abs;

    ctrl_mode = 2.0;

    if (t - t_circle0) >= T_circle
        q_hold(1) = q_target(1);
        q_hold(2) = q_target(2);
        q_hold(3) = q_target(3);
        state = 3;
    end
end

%% ============================================================
% State 3: stop / hold final pose
% ============================================================

if state == 3

    q_target(1) = q_hold(1);
    q_target(2) = q_hold(2);
    q_target(3) = q_hold(3);

    v_ref = 0.0;
    omega_ref = 0.0;
    ctrl_mode = 3.0;
    elapsed_circle = T_circle;
end

%% ============================================================
% Outputs
% ============================================================

state_out = state;
task3_active = double(state == 2);
task3_done = double(state == 3);

end

function a = wrapToPiLocal(a)
a = atan2(sin(a), cos(a));
end

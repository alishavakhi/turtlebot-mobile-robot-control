% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_151.xml) by tools/extract_model_code.py
function [q_d, dq_d, u_d] = task_manager_linear(q_est, q_goal1, q_goal2, t)
%TASK_MANAGER_LINEAR
% State-machine task manager for the linear trajectory tracking controller.
%
% Outputs:
%   q_d  = desired pose [x_d; y_d; theta_d]
%   dq_d = desired inertial velocity [dx_d; dy_d; dtheta_d]
%   u_d  = desired unicycle input [v_d; omega_d]

persistent state t_reached theta_hold

if isempty(state)
    state = 1;
    t_reached = 0;
    theta_hold = q_est(3);
end

%% Parameters
v_ref = 0.16;
v_min = 0.04;
lookahead = 0.45;

wp_tol = 0.14;
goal_tol = 0.06;

k_theta = 1.2;
omega_max = 1.2;

%% Waypoints
q_wp1 = [-0.20; -1.10; 0.0];
q_wp2 = [-0.20; -0.30; pi/2];
q_wp3 = [1.45; -0.30; 0.0];

q_start_task3 = [2.00; -0.60; 0.0];

%% Circle parameters
R_circle = 0.60;
omega_circle = 0.35;
look_ahead_time = 0.4;
circle_duration = 28.0;

%% State machine target selection
target = q_goal1;
mode = 0;  % 0 point target, 1 circle, 2 hold

switch state
    case 1
        target = q_goal1;
        if pos_dist(q_est, q_goal1) < goal_tol
            state = 2;
            t_reached = t;
            theta_hold = q_goal1(3);
        end

    case 2
        target = q_goal1;
        mode = 2;
        if (t - t_reached) >= 1.0
            state = 3;
        end

    case 3
        target = q_wp1;
        if pos_dist(q_est, q_wp1) < wp_tol
            state = 4;
        end

    case 4
        target = q_wp2;
        if pos_dist(q_est, q_wp2) < wp_tol
            state = 5;
        end

    case 5
        target = q_goal2;
        if pos_dist(q_est, q_goal2) < goal_tol
            state = 6;
            t_reached = t;
        end

    case 6
        target = q_wp3;
        if pos_dist(q_est, q_wp3) < wp_tol
            state = 7;
        end

    case 7
        target = q_start_task3;
        if pos_dist(q_est, q_start_task3) < goal_tol
            state = 8;
            t_reached = t;
            theta_hold = q_start_task3(3);
        end

    case 8
        target = q_start_task3;
        mode = 2;
        if (t - t_reached) >= 1.0
            state = 9;
            t_reached = t;
        end

    case 9
        mode = 1;

    otherwise
        target = q_goal1;
end

%% Generate reference
if mode == 1
    elapsed = (t - t_reached) + look_ahead_time;
    elapsed = min(elapsed, circle_duration);

    x_d = q_start_task3(1) + R_circle*sin(omega_circle*elapsed);
    y_d = q_start_task3(2) + R_circle*(1 - cos(omega_circle*elapsed));

    dx_d = R_circle*omega_circle*cos(omega_circle*elapsed);
    dy_d = R_circle*omega_circle*sin(omega_circle*elapsed);

    theta_d = atan2(dy_d, dx_d);
    v_d = sqrt(dx_d^2 + dy_d^2);
    omega_d = omega_circle;

    q_d = [x_d; y_d; theta_d];
    dq_d = [dx_d; dy_d; omega_d];
    u_d = [v_d; omega_d];

elseif mode == 2
    q_d = [target(1); target(2); theta_hold];
    dq_d = [0; 0; 0];
    u_d = [0; 0];

else
    dx = target(1) - q_est(1);
    dy = target(2) - q_est(2);
    dist = hypot(dx, dy);

    theta_d = atan2(dy, dx);
    theta_err = wrap_pi(theta_d - q_est(3));

    speed_scale_dist = min(1.0, dist/lookahead);
    speed_scale_heading = max(0.25, cos(theta_err));

    v_d = v_ref * speed_scale_dist * speed_scale_heading;

    if dist > goal_tol
        v_d = max(v_d, v_min);
    else
        v_d = 0;
    end

    omega_d = k_theta * theta_err;
    omega_d = saturate(omega_d, omega_max);

    q_d = [target(1); target(2); theta_d];
    dq_d = [v_d*cos(theta_d); v_d*sin(theta_d); omega_d];
    u_d = [max(v_d, 1e-3); omega_d];

    if dist < goal_tol
        q_d = [target(1); target(2); target(3)];
        dq_d = [0; 0; 0];
        u_d = [0; 0];
    end
end

end

function d = pos_dist(q, p)
d = hypot(q(1)-p(1), q(2)-p(2));
end

function a = wrap_pi(a)
a = atan2(sin(a), cos(a));
end

function y = saturate(x, lim)
y = max(min(x, lim), -lim);
end

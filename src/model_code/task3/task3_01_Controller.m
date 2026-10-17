% Extracted verbatim from task3_simulation.slx (simulink/stateflow/chart_30.xml) by tools/extract_model_code.py
function [v, omega] = Controller(q_est, q_target, k_x, k_y, k_theta, v_ref, omega_ref, ctrl_mode)
%#codegen
% Task 3 controller:
% ctrl_mode = 1: regulation to Task 3 start point
% ctrl_mode = 2: circular trajectory tracking
% ctrl_mode = 3: stop

theta = q_est(3);

dx = q_target(1) - q_est(1);
dy = q_target(2) - q_est(2);
dtheta = wrapToPiLocal(q_target(3) - q_est(3));

%% Safety limits
v_lim = 0.22;
omega_lim = 0.60;

%% ============================================================
% Mode 2: circular trajectory tracking
% ============================================================

if ctrl_mode == 2

    % Error expressed in robot frame
    e1 =  cos(theta)*dx + sin(theta)*dy;
    e2 = -sin(theta)*dx + cos(theta)*dy;
    e3 = dtheta;

    if abs(e3) < 1e-4
        sinc_e3 = 1.0;
    else
        sinc_e3 = sin(e3)/e3;
    end

    % Tuned circular tracking gains
    k1 = 0.70;
    k2 = 2.30;
    k3 = 0.7;

    v = v_ref*cos(e3) + k1*e1;

    omega = omega_ref ...
          + k2*v_ref*sinc_e3*e2 ...
          + k3*e3;

%% ============================================================
% Mode 1: regulation to the circular start point
% ============================================================

elseif ctrl_mode == 1

    rho = sqrt(dx^2 + dy^2);
    angle_to_goal = atan2(dy, dx);
    alpha = wrapToPiLocal(angle_to_goal - theta);

    % Regulation gains
    k_rho   = 0.55;
    k_alpha = 1.20;
    k_final = 0.95;

    % Important:
    % rho_switch must be smaller than pos_thr_start in Task_Manager.
    % Task_Manager pos_thr_start = 0.060
    rho_switch = 0.005;

    if rho > rho_switch

        v = k_rho*rho*cos(alpha);

        % If the robot is badly oriented toward the point, avoid cutting.
        if abs(alpha) > 1.2
            v = 0.0;
        elseif abs(alpha) > 0.6
            v = 0.4*v;
        end

        omega = k_alpha*alpha;

    else

        % Close to the start point:
        % stop translation and align heading with the circle tangent.
        v = 0.0;
        omega = k_final*dtheta;

    end

%% ============================================================
% Mode 3: stop
% ============================================================

else

    v = 0.0;
    omega = 0.0;

end

%% Saturation
v = satLocal(v, -v_lim, v_lim);
omega = satLocal(omega, -omega_lim, omega_lim);

end

function y = satLocal(u, umin, umax)
y = min(max(u, umin), umax);
end

function a = wrapToPiLocal(a)
a = atan2(sin(a), cos(a));
end

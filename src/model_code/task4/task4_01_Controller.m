% Extracted verbatim from task4_simulation.slx (simulink/stateflow/chart_30.xml) by tools/extract_model_code.py
function [v, omega] = Controller(q_est, q_target, k_x, k_y, k_theta, v_ref, omega_ref, ctrl_mode)
%#codegen
% Tracking Controller
% ctrl_mode = 1: approach/regulation
% ctrl_mode = 2: trajectory tracking
% ctrl_mode = 3: stop

theta = q_est(3);

dx = q_target(1) - q_est(1);
dy = q_target(2) - q_est(2);

dtheta = wrapToPiLocal(q_target(3) - q_est(3));

%% Limits
v_lim = 0.22;
omega_lim = 0.80;

%% Mode 2: trajectory tracking
if ctrl_mode == 2

    ex =  cos(theta)*dx + sin(theta)*dy;
    ey = -sin(theta)*dx + cos(theta)*dy;
    eth = dtheta;

    k1 = 0.75;
    k2 = 2.00;
    k3 = 1.20;

    v = v_ref*cos(eth) + k1*ex;
    omega = omega_ref + k2*v_ref*ey + k3*sin(eth);

%% Mode 1: approach to circle start
elseif ctrl_mode == 1

    rho = sqrt(dx^2 + dy^2);
    angle_to_goal = atan2(dy, dx);
    alpha = wrapToPiLocal(angle_to_goal - theta);

    k_rho = 0.55;
    k_alpha = 1.20;
    k_final = 0.95;

    rho_switch = 0.010;

    if rho > rho_switch

        v = k_rho*rho*cos(alpha);

        if abs(alpha) > 1.2
            v = 0.0;
        elseif abs(alpha) > 0.6
            v = 0.4*v;
        end

        omega = k_alpha*alpha;

    else

        v = 0.0;
        omega = k_final*dtheta;

    end

%% Mode 3: stop
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

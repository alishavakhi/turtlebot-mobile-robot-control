% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_46.xml) by tools/extract_model_code.py
function q_est = fcn_exact(v, omega, q_init, Ts)
    persistent q_prev;
    
    if isempty(q_prev)
        q_prev = q_init;
    end
    
    theta_prev = q_prev(3);
    theta_new = wrapToPi(theta_prev + omega * Ts);
    
    % Use a small tolerance to avoid division by zero when going straight
    if abs(omega) < 1e-6
        x_new = q_prev(1) + v * cos(theta_prev) * Ts;
        y_new = q_prev(2) + v * sin(theta_prev) * Ts;
    else
        x_new = q_prev(1) + (v / omega) * (sin(theta_prev + omega*Ts) - sin(theta_prev));
        y_new = q_prev(2) - (v / omega) * (cos(theta_prev + omega*Ts) - cos(theta_prev));
    end
    
    q_est = [x_new; y_new; theta_new];
    q_prev = q_est;
end

% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_210.xml) by tools/extract_model_code.py
function q_est = fcn(v, omega, q_init, Ts)
    % Persistent variables hold their value between function calls
    persistent q_prev;
    
    if isempty(q_prev)
        q_prev = q_init;
    end
    
    % Dead-reckoning equations
    theta_new = q_prev(3) + omega * Ts;
    
    % Ensure angle is wrapped between -pi and pi
    theta_new = wrapToPi(theta_new);
    
    x_new = q_prev(1) + v * cos(theta_new) * Ts;
    y_new = q_prev(2) + v * sin(theta_new) * Ts;
    
    q_est = [x_new; y_new; theta_new];
    q_prev = q_est;
end

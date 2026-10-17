% Extracted verbatim from task1_simulation.slx (simulink/stateflow/chart_16.xml) by tools/extract_model_code.py
function [v, omega] = fcn(q_est, q_target, k1, k2, k_theta)
    % Default outputs
    v = 0;
    omega = 0;
    
    % Internal safety limits
    v_limit = 0.22;       % linear velocity limit [m/s]
    omega_limit = 0.6;    % body angular velocity limit [rad/s]
                          % 10 rad/s is wheel-speed limit, not body omega
    
    % Position error
    dx = q_target(1) - q_est(1);
    dy = q_target(2) - q_est(2);
    
    rho = sqrt(dx^2 + dy^2);
    
    % ------------------------------------------------------------
    % Near target position: rotate in place to correct orientation
    % ------------------------------------------------------------
    if rho < 0.001
        e_theta = atan2(sin(q_target(3) - q_est(3)), ...
                        cos(q_target(3) - q_est(3)));
        
        v = 0;
        omega = k_theta * e_theta;
        
        % Saturate body angular velocity
        if abs(omega) > omega_limit
            omega = omega_limit * sign(omega);
        end
        
        % Stop only when orientation is also reached
        if abs(e_theta) < 0.05
            omega = 0;
        end
        
        return;
    end
    
    % Polar error variables
    theta_target = atan2(dy, dx);
    
    gamma = atan2(sin(theta_target - q_est(3)), ...
                  cos(theta_target - q_est(3)));
              
    delta = atan2(sin(gamma + q_est(3) - q_target(3)), ...
                  cos(gamma + q_est(3) - q_target(3)));
    
    % Astolfi posture regulation control law
    v = k1 * rho * cos(gamma);
    
    if abs(gamma) < 1e-4
        sinc_gamma = 1;
    else
        sinc_gamma = sin(gamma) / gamma;
    end
    
    omega = k2 * gamma + k1 * sinc_gamma * cos(gamma) * ...
            (gamma + k_theta * delta);
    
    % Saturation
    if abs(v) > v_limit
        v = v_limit * sign(v);
    end
    
    if abs(omega) > omega_limit
        omega = omega_limit * sign(omega);
    end
end

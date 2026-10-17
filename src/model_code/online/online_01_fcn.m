% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_34.xml) by tools/extract_model_code.py
function [v, omega] = fcn(q_est, q_target, k1, k2, k3)

    % Default outputs
    v = 0;
    omega = 0;
    
    % --- Internal Settings (instead of external constants) ---
    v_limit = 0.22;    % Maximum linear velocity [m/s]
    omega_limit = 10; % Maximum angular velocity according to project requirements [rad/s]
    
    % 1. Compute errors in polar coordinates
    dx = q_target(1) - q_est(1);
    dy = q_target(2) - q_est(2);
    
    rho = sqrt(dx^2 + dy^2);
    theta_target = atan2(dy, dx);
    
    gamma = wrapToPi(theta_target - q_est(3));
    delta = wrapToPi(gamma + q_est(3) - q_target(3));
    
    % 2. Control Law (Astolfi Controller)
    if rho < 0.05
        
        % Stop near the target
        v = 0;
        omega = 0;
        
    else
        
        % Compute linear velocity
        v = k1 * rho * cos(gamma);
        
        % Handle sinc(gamma) to avoid division by zero
        if abs(gamma) < 1e-4
            sinc_gamma = 1;
        else
            sinc_gamma = sin(gamma) / gamma;
        end
        
        % Compute angular velocity
        omega = k2 * gamma + ...
                k1 * sinc_gamma * cos(gamma) * (gamma + k3 * delta);
    end
    
    % 3. Apply saturation limits directly in the code
    if abs(v) > v_limit
        v = v_limit * sign(v);
    end
    
    if abs(omega) > omega_limit
        omega = omega_limit * sign(omega);
    end
end

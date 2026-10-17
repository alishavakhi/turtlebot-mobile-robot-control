% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_77.xml) by tools/extract_model_code.py
function [q_est, q_pred] = fcn(v, omega, Y_vicon, Ts, q_init, is_real_robot)

    % Extended Kalman Filter with mode-aware initialization
    persistent q_hat P Q initialized
    q_est  = zeros(3,1);
    q_pred = zeros(3,1);
    
    % --- 1. Smart Initialization ---
    if isempty(initialized)
        
        if is_real_robot == 1
            
            % Real robot mode:
            % Wait until valid Vicon data becomes available
            if all(Y_vicon == 0)
                q_est = q_init; % Return initial estimate until data arrives
                q_pred = q_init;
                return;
            else
                q_hat = Y_vicon; % Lock onto the first valid Vicon measurement
                initialized = true;
            end
            
        else
            
            % Simulation mode:
            % Start immediately using q_init
            q_hat = q_init;
            initialized = true;
        end
        
        P = eye(3) * 0.1; 
        
        % Process noise covariance
        % High confidence in odometry model
        Q = diag([1e-4, 1e-4, 5e-4]);
    end

    % --- 2. Prediction Step ---
    
    theta = q_hat(3);
    
    % Nonlinear unicycle kinematic model
    q_pred = zeros(3,1);
    q_pred = q_hat + [v*cos(theta); 
                       v*sin(theta); 
                       omega] * Ts;
                   
    q_pred(3) = wrapToPi(q_pred(3));

    % Jacobian matrix (linearization)
    F = [1, 0, -v*sin(theta)*Ts;
         0, 1,  v*cos(theta)*Ts;
         0, 0,  1];

    P = F * P * F' + Q;

    % --- 3. Update Step ---
    
    % Detect red zone (high sensor noise region)
    in_red = (q_pred(1) >= -0.6 && q_pred(1) <= 0.2);
    
    % Check if Vicon signal is lost
    is_lost = all(Y_vicon == 0);

    if in_red
        
        % High measurement noise covariance inside noisy region
        R = diag([0.7, 0.7, 0.9]);
        
    else
        
        % Low measurement noise covariance in safe region
        R = diag([1e-5, 1e-5, 5e-5]);
    end

    if is_lost
        
        % Task 3:
        % Sensor unavailable, rely only on odometry prediction
        q_est = q_pred;
        
    else
        
        % Standard Kalman Filter update
        
        H = eye(3);
        
        K = (P * H') / (H * P * H' + R);
        
        innovation = Y_vicon - q_pred;
        
        % Correct angular innovation wrapping
        innovation(3) = wrapToPi(innovation(3));
        
        q_est = q_pred + K * innovation;
        q_est(3) = wrapToPi(q_est(3));

        % Covariance update
        P = (eye(3) - K * H) * P;
    end
    
    % Store updated estimate for next iteration
    q_hat = q_est;
end

% Extracted verbatim from task3_simulation.slx (simulink/stateflow/chart_82.xml) by tools/extract_model_code.py
function q_est = fcn(v, omega, Y_vicon, Ts, q_init, is_real_robot)
% EKF-based localizer for TurtleBot Task 2
%
% Prediction:
%   Unicycle model using odometric inputs v and omega.
%
% Update:
%   Vicon pose measurement when available.
%
% In the red area, the Vicon measurement covariance is increased,
% so the EKF trusts the odometric prediction more than the noisy Vicon.

persistent q_hat P Q initialized

%% ============================================================
% 1. Initialization
% ============================================================

if isempty(initialized)

    if is_real_robot == 1
        % Real robot mode:
        % Wait until a valid Vicon measurement is available.
        if all(Y_vicon == 0)
            q_est = q_init;
            return;
        else
            q_hat = Y_vicon;
            q_hat(3) = localWrapToPi(q_hat(3));
            initialized = true;
        end

    else
        % Simulation mode:
        % Start directly from the selected initial condition.
        q_hat = q_init;
        q_hat(3) = localWrapToPi(q_hat(3));
        initialized = true;
    end

    % Initial covariance
    P = eye(3) * 0.1;

    % Process noise covariance:
    % uncertainty of the odometric/unicycle prediction.
    Q = diag([1e-4, 1e-4, 5e-4]);
end

%% ============================================================
% 2. Prediction step
% ============================================================

theta = q_hat(3);

% Unicycle model prediction
q_pred = q_hat + [v*cos(theta); ...
                  v*sin(theta); ...
                  omega] * Ts;

q_pred(3) = localWrapToPi(q_pred(3));

% Jacobian of the prediction model with respect to the state
F = [1, 0, -v*sin(theta)*Ts;
     0, 1,  v*cos(theta)*Ts;
     0, 0,  1];

% Covariance prediction
P = F * P * F' + Q;

%% ============================================================
% 3. Measurement validity and red-area detection
% ============================================================

% Missing Vicon measurement is represented as [0; 0; 0]
is_lost = all(Y_vicon == 0);

% Red noisy Vicon area:
% x5 <= x <= x6 and y1 <= y <= y6
in_red = (q_pred(1) >= -0.60 && q_pred(1) <= 0.20 && ...
          q_pred(2) >= -1.50 && q_pred(2) <= 0.10);

%% ============================================================
% 4. Measurement covariance
% ============================================================

if in_red
    % In the red area, Vicon is noisy.
    % Therefore, the filter trusts the Vicon correction less.
    R = diag([0.7, 0.7, 0.9]);
else
    % Outside the red area, Vicon is assumed to be reliable.
    R = diag([1e-5, 1e-5, 5e-5]);
end

%% ============================================================
% 5. Update step
% ============================================================

if is_lost
    % If Vicon is not available, use only the prediction.
    q_est = q_pred;

else
    % Measurement model:
    % Vicon directly measures [x; y; theta]
    H = eye(3);

    % Kalman gain
    K = (P * H') / (H * P * H' + R);

    % Innovation
    innovation = Y_vicon - q_pred;
    innovation(3) = localWrapToPi(innovation(3));

    % State correction
    q_est = q_pred + K * innovation;
    q_est(3) = localWrapToPi(q_est(3));

    % Covariance correction
    P = (eye(3) - K * H) * P;
end

% Store estimate for next sample
q_hat = q_est;

end

%% ============================================================
% Local angle wrapping function
% ============================================================

function a = localWrapToPi(a)
a = atan2(sin(a), cos(a));
end

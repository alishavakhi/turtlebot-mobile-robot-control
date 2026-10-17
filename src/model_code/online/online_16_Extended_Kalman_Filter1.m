% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_188.xml) by tools/extract_model_code.py
function [P_KF_new, z_KF_new] = Extended_Kalman_Filter1(P_KF, z_KF, u_wheels, measurements, D, R_5,r, d, rnd, gaussian_noise, T_s,index, R_2,R_bad, R_good)

% z_KF = [x_EKF, y_EKF, theta_EKF, delta_phi_L_EKF, delta_phi_R_EKF, omega_L_EKF, omega_R_EKF]

v_k = r * (z_KF(6) + z_KF(7))/2;
w_k = r * (z_KF(7) - z_KF(6))/d;

theta_mean = z_KF(3) + (w_k * T_s)/2;

F_k = [1, 0, -T_s * v_k * sin(theta_mean), 0, 0, T_s * r * cos(theta_mean)/2 + (T_s)^2 * r * v_k * sin(theta_mean)/(2*d), T_s * r * cos(theta_mean)/2 - (T_s)^2 * r * v_k * sin(theta_mean)/(2*d);
    0, 1, T_s * v_k * cos(theta_mean), 0, 0, T_s * r * sin(theta_mean)/2 - (T_s)^2 * r * v_k * cos(theta_mean)/(2*d), T_s * r * sin(theta_mean)/2 + (T_s)^2 * r * v_k * cos(theta_mean)/(2*d);
    0, 0, 1, 0, 0, (-T_s*r)/d, (T_s*r)/d;
    0, 0, 0, 1, 0, T_s, 0;
    0, 0, 0, 0, 1, 0, T_s;
    0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 1];

%% Prediction part
% theta_mid = z_KF(3) + 0.5*w_k*T_s;

% z_pred = [
%     z_KF(1) + T_s*v_k*cos(theta_mid);
%     z_KF(2) + T_s*v_k*sin(theta_mid);
%     z_KF(3) + T_s*w_k;
%     z_KF(4) + T_s*z_KF(6);
%     z_KF(5) + T_s*z_KF(7);
%     u_wheels(1);
%     u_wheels(2)
% ];

z_pred = [z_KF(1)+(T_s*r*(z_KF(6)+z_KF(7))/2)*cos(theta_mean + (T_s*r*(z_KF(6)+z_KF(7)))/(2*d));
    z_KF(2)+(T_s*r*(z_KF(6)+z_KF(7))/2)*sin(theta_mean + (T_s*r*(z_KF(7)-z_KF(6)))/(2*d));
    theta_mean + (T_s*r*(z_KF(7)-z_KF(6))/(2*d));
    z_KF(4) + u_wheels(1);
    z_KF(5) + u_wheels(2);
    u_wheels(1);
    u_wheels(2)];

P_pred = F_k * P_KF * F_k' + D;


%% Update

% initialize dimensions
y_measurement = zeros(6,1);
H = zeros(5,7);
R = R_5;  % initialize with largest

m = 0; % actual measurement dimension
if index == 2
    % Robot is INSIDE the zone: High chance of total signal loss
    prob_gps_loss = 0.9; 
else
    % Robot is OUTSIDE the zone: Perfect or near-perfect reception
    prob_gps_loss = 0.0; % Or set to 0.05 if you want occasional random drops anywhere
end

if rnd < prob_gps_loss

     % ONLY ENCODERS 
    m = 2;
    
    y_measurement(1:2) = [
        measurements(4);
        measurements(5)
    ];

    H(1:2,:) = [
        0 0 0 1 0 0 0;
        0 0 0 0 1 0 0
    ];

    R = R_2;

    

else
            
        % FULL SENSING
        m = 5;
        
        % MoCap + ENCODERS + IMU 
        y_measurement(1:5) = [
            measurements(1) ;  % x
            measurements(2) ;  % y
            measurements(3) ;  % θ
            measurements(4);                   % Δφ_L
            measurements(5);                   % Δφ_R
        ];

        H(1:5,:) = [
            1 0 0 0 0 0 0;
            0 1 0 0 0 0 0;
            0 0 1 0 0 0 0;
            0 0 0 1 0 0 0;
            0 0 0 0 1 0 0;
           
        ];
      

        % Dynamically adjust measurement confidence based on the noise zone
        if index == 2
            % Inside the red zone: Sensors are noisy, trust the prediction more
            R = R_bad; 
        else
            % Outside the red zone: Sensors are clean, trust the measurements
            R = R_good; 
        end
        % R = R_5;


         
end


% Trim to actual sizes
y_measurement = y_measurement(1:m);
H = H(1:m, :);
R = R(1:m, 1:m);

err = y_measurement - H * z_pred;
S = H * P_pred * H' + R;
K = P_pred * H' / S;

% states update
z_KF_new = z_pred + K * err;

% covariance update
I = eye(size(P_pred));
P_KF_new = (I - K*H)*P_pred*(I - K*H)' + K*R*K';

% Normalize angle
z_KF_new(3) = atan2(sin(z_KF_new(3)), cos(z_KF_new(3)));

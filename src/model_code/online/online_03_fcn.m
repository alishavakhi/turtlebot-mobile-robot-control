% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_56.xml) by tools/extract_model_code.py
function Y_vicon_fake = fcn(q_real)

    % q_real is the odometry output,
    % assumed here to represent the robot's true position
    
    x = q_real(1);
    y = q_real(2);
    theta = q_real(3);
    
    % Simulate sensor noise inside the red zone
    % (between x = -0.6 and x = 0.2)
    if (x >= -0.6 && x <= 0.2)
        
        % Add Gaussian noise
        noise = 0.02 * randn(3,1);
        
        Y_vicon_fake = q_real + noise;
        
    else
        
        % Outside the red zone,
        % the sensor is assumed to be perfectly accurate
        Y_vicon_fake = q_real;
    end
end

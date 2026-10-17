% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_117.xml) by tools/extract_model_code.py
function q_target = fcn(q_est, q_goal1, q_goal2, t)
    coder.extrinsic('disp');    

    % Default initialization to avoid Simulink Coder errors
    q_target = q_goal1; 
    
    % Define persistent variables to store state and timing information
    persistent state t_reached
    if isempty(state)
        state = 1; % Robot starts from Task 1
        t_reached = 0; % Stores the time when each target is reached
    end

    % --- Intermediate Waypoints for Collision-Free Navigation ---
    q_wp1 = [-0.20; -1.10; 0];    % First waypoint: pass below the gray obstacle
    q_wp2 = [-0.20; -0.30; pi/2]; % Second waypoint: rotate upward toward the corridor
    q_wp3 = [1.45; -0.30; 0];     % Third waypoint: exit corridor and prepare for Task 3
    
    % --- Starting Point of Task 3 (Circle Path) ---
    q_start_task3 = [2.00; -0.60; 0]; % Positioned below the brown obstacle for circle alignment

    % --- Task 3 Parameters (Circular Motion) ---
    R_circle = 0.60;      % Circle radius chosen for safe obstacle avoidance
    omega_circle = 0.35;  % Angular speed kept low to reduce Kalman filter drift

    % --- Euclidean Distance Calculations ---
    dist1 = sqrt((q_est(1)-q_goal1(1))^2 + (q_est(2)-q_goal1(2))^2);
    dist_wp1 = sqrt((q_est(1)-q_wp1(1))^2 + (q_est(2)-q_wp1(2))^2);
    dist_wp2 = sqrt((q_est(1)-q_wp2(1))^2 + (q_est(2)-q_wp2(2))^2);
    dist2 = sqrt((q_est(1)-q_goal2(1))^2 + (q_est(2)-q_goal2(2))^2);
    dist_wp3 = sqrt((q_est(1)-q_wp3(1))^2 + (q_est(2)-q_wp3(2))^2);
    dist_start3 = sqrt((q_est(1)-q_start_task3(1))^2 + (q_est(2)-q_start_task3(2))^2);

    % --- Target Reaching Thresholds ---
    thresh_strict = 0.05; % Strict threshold for final targets
    thresh_loose = 0.15;  % Looser threshold for smooth waypoint transitions

    % --- State Machine Logic ---
    switch state
        
        case 1 % Move toward Task 1
            q_target = q_goal1;

            if dist1 < thresh_strict
                state = 2; 
                t_reached = t; % Save arrival time for timer
                fprintf('Goal 1 Reached\n');
            end
            
        case 2 % Wait 1 second at Task 1
            q_target = q_goal1;

            if (t - t_reached) >= 1.0
                state = 3;
                fprintf('Moving to Waypoint 1\n');
            end
            
        case 3 % Move toward Waypoint 1
            q_target = q_wp1;

            if dist_wp1 < thresh_loose
                state = 4;
                fprintf('Waypoint 1 Cleared\n');
            end
            
        case 4 % Move toward Waypoint 2
            q_target = q_wp2;

            if dist_wp2 < thresh_loose
                state = 5;
                fprintf('Waypoint 2 Cleared\n');
            end
            
        case 5 % Move toward Task 2
            q_target = q_goal2;

            if dist2 < thresh_strict
                state = 6; 
                fprintf('Goal 2 Reached\n');
            end
            
        case 6 % Move toward Waypoint 3 before starting circle motion
            q_target = q_wp3;

            if dist_wp3 < thresh_loose
                state = 7;
                fprintf('Waypoint 3 Cleared\n');
            end
            
        case 7 % Move toward Task 3 starting point
            q_target = q_start_task3;

            if dist_start3 < thresh_strict
                state = 8; 
                t_reached = t; % Save arrival time for Task 3 timer
                fprintf('Task 3 Start Point Reached\n');
            end
            
        case 8 % Wait 1 second before starting circular motion
            q_target = q_start_task3;

            if (t - t_reached) >= 1.0
                state = 9; 
                t_reached = t; % Reset timer for circular trajectory
                fprintf('Starting Circle\n');
            end
            
        case 9 % Task 3: Follow circular trajectory around brown obstacle
            
            % Virtual rabbit technique:
            % Move the target slightly ahead to prevent spinning in place
            look_ahead = 0.4;
            elapsed = (t - t_reached) + look_ahead;
            
            % Stop after approximately 1.5 revolutions
            if elapsed > 28
                elapsed = 28; 
            end
            
            % Shifted circle equation to ensure smooth tangent entry
            x_d = q_start_task3(1) + R_circle * sin(omega_circle * elapsed);
            y_d = q_start_task3(2) + R_circle * (1 - cos(omega_circle * elapsed));

            % Continuous orientation update
            theta_d = wrapToPi(omega_circle * elapsed);
            
            q_target = [x_d; y_d; theta_d];
            
        otherwise % Default fallback state to prevent Simulink crash
            q_target = q_goal1;
    end
end

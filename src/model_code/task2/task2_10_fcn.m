% Extracted verbatim from task2_simulation.slx (simulink/stateflow/chart_98.xml) by tools/extract_model_code.py
function q_target = fcn(q_est, q_goal1, q_goal2, t)
%#codegen
% Robust Task 2 waypoint manager.
% q_goal1 is kept only to preserve Simulink wiring.
%
% Target sequence:
%   waypoint 1 -> waypoint 2 -> final goal

persistent state t_state

if isempty(state) || t <= 0.04
    state = 1;
    t_state = t;
end

% ------------------------------------------------------------
% Task 2 waypoints
% ------------------------------------------------------------
q_wp1 = [-0.20; -1.10; 0.0];
q_wp2 = [-0.20; -0.30; pi/2];

q_goal = q_goal2;

% ------------------------------------------------------------
% Switching thresholds
% ------------------------------------------------------------
thresh_wp1_pos  = 0.10;
thresh_wp2_pos  = 0.08;
thresh_wp2_ang  = 0.45;   % rad, about 25.8 deg
thresh_goal_pos = 0.05;

% Safety timeout only to avoid getting stuck forever at wp2
max_wp2_hold_time = 4.0;

% ------------------------------------------------------------
% Distance errors
% ------------------------------------------------------------
dist_wp1 = sqrt((q_est(1) - q_wp1(1))^2 + ...
                (q_est(2) - q_wp1(2))^2);

dist_wp2 = sqrt((q_est(1) - q_wp2(1))^2 + ...
                (q_est(2) - q_wp2(2))^2);

dist_goal = sqrt((q_est(1) - q_goal(1))^2 + ...
                 (q_est(2) - q_goal(2))^2);

theta_err_wp2 = localWrapToPi(q_wp2(3) - q_est(3));

% ------------------------------------------------------------
% State machine
% ------------------------------------------------------------
switch state

    case 1
        % Move to waypoint 1
        q_target = q_wp1;

        if dist_wp1 < thresh_wp1_pos
            state = 2;
            t_state = t;
        end

    case 2
        % Move to waypoint 2 and do not switch too early.
        % The robot should be close to wp2 and reasonably aligned
        % before moving to the final goal.
        q_target = q_wp2;

        enough_position = dist_wp2 < thresh_wp2_pos;
        enough_heading  = abs(theta_err_wp2) < thresh_wp2_ang;
        timeout_reached = (t - t_state) > max_wp2_hold_time;

        if enough_position && (enough_heading || timeout_reached)
            state = 3;
            t_state = t;
        end

    case 3
        % Move to final goal
        q_target = q_goal;

        if dist_goal < thresh_goal_pos
            state = 4;
            t_state = t;
        end

    case 4
        % Hold final goal
        q_target = q_goal;

    otherwise
        state = 1;
        t_state = t;
        q_target = q_wp1;
end

end

function a = localWrapToPi(a)
    a = atan2(sin(a), cos(a));
end

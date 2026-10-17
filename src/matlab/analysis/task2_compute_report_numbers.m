clear; clc; close all;

%% ============================================================
% Compute Task 2 report numbers
%
% Required files:
%   test3_task2_full_outputs.mat
%   exported_task2_simulation.mat
%
% Prints:
%   - durations
%   - final errors
%   - localization errors
%   - red-area localization errors
%   - control input ranges
%   - wheel-speed ranges and saturation check
%   - obstacle clearance check
% ============================================================

%% --------- FILES ---------
realFile = 'test3_task2_full_outputs.mat';
simFile  = 'exported_task2_simulation.mat';

if ~isfile(realFile)
    error('Real Task 2 file not found: %s', realFile);
end

if ~isfile(simFile)
    error('Simulation Task 2 file not found: %s', simFile);
end

Sreal = load(realFile);
Ssim  = load(simFile);

Dreal = Sreal.D_task2;
Dsim  = Ssim.Dsim_task2;

%% --------- CONSTANTS ---------
Ts = 0.04;
goal = [1.00, -0.30, 0];

red_x_min = -0.60;
red_x_max =  0.20;
red_y_min = -1.50;
red_y_max =  0.10;

omega_limit = 10;  % rad/s

% Obstacles from the map:
% [x_min, y_min, width, height]
obstacles = [
    -1.50, -0.70, 0.90, 0.80;   % left obstacle: x -1.50 to -0.60, y -0.70 to 0.10
     0.20, -1.50, 0.80, 0.80;   % lower obstacle: x 0.20 to 1.00, y -1.50 to -0.70
     1.80, -0.20, 0.40, 0.40    % cube/orange obstacle
];

map_x_min = -2.90;
map_x_max =  2.90;
map_y_min = -1.50;
map_y_max =  1.50;

%% --------- EXTRACT REAL SIGNALS ---------
q_real_est = forceCols(Dreal.q_est, 3);
q_real_target = forceCols(Dreal.q_target, 3);

if isfield(Dreal, 'Y_viscon_ground_trith')
    q_real_vicon = forceCols(Dreal.Y_viscon_ground_trith, 3);
elseif isfield(Dreal, 'Y_vicon_ground_truth')
    q_real_vicon = forceCols(Dreal.Y_vicon_ground_truth, 3);
elseif isfield(Dreal, 'Y_viscon')
    q_real_vicon = forceCols(Dreal.Y_viscon, 3);
else
    q_real_vicon = q_real_est;
    warning('No real Vicon ground truth found. Using q_est as q_real_vicon.');
end

if isfield(Dreal, 't')
    t_real = Dreal.t(:);
elseif isfield(Dreal, 'tout')
    t_real = Dreal.tout(:);
else
    t_real = (0:size(q_real_est,1)-1).' * Ts;
end
t_real = t_real - t_real(1);

if isfield(Dreal, 'v')
    v_real = Dreal.v(:);
else
    v_real = NaN(size(t_real));
end

if isfield(Dreal, 'omega')
    omega_real = Dreal.omega(:);
else
    omega_real = NaN(size(t_real));
end

if isfield(Dreal, 'omega_wheels')
    omega_wheels_real_cmd = forceCols(Dreal.omega_wheels, 2);
else
    omega_wheels_real_cmd = NaN(numel(t_real),2);
end

if isfield(Dreal, 'bot_wheelspeed')
    omega_wheels_real_meas = forceCols(Dreal.bot_wheelspeed, 2);
else
    omega_wheels_real_meas = NaN(numel(t_real),2);
end

%% --------- EXTRACT SIMULATION SIGNALS ---------
q_sim_actual = forceCols(Dsim.q_actual, 3);
q_sim_est    = forceCols(Dsim.q_est, 3);
q_sim_target = forceCols(Dsim.q_target, 3);

if isfield(Dsim, 't')
    t_sim = Dsim.t(:);
elseif isfield(Dsim, 'tout')
    t_sim = Dsim.tout(:);
else
    t_sim = (0:size(q_sim_actual,1)-1).' * Ts;
end
t_sim = t_sim - t_sim(1);

if isfield(Dsim, 'v')
    v_sim = Dsim.v(:);
else
    v_sim = NaN(size(t_sim));
end

if isfield(Dsim, 'omega')
    omega_sim = Dsim.omega(:);
else
    omega_sim = NaN(size(t_sim));
end

if isfield(Dsim, 'omega_wheels')
    omega_wheels_sim_cmd = forceCols(Dsim.omega_wheels, 2);
else
    omega_wheels_sim_cmd = NaN(numel(t_sim),2);
end

%% --------- ANGLE WRAPPING ---------
q_real_vicon(:,3)  = wrapToPiLocal(q_real_vicon(:,3));
q_real_est(:,3)    = wrapToPiLocal(q_real_est(:,3));
q_real_target(:,3) = wrapToPiLocal(q_real_target(:,3));

q_sim_actual(:,3)  = wrapToPiLocal(q_sim_actual(:,3));
q_sim_est(:,3)     = wrapToPiLocal(q_sim_est(:,3));
q_sim_target(:,3)  = wrapToPiLocal(q_sim_target(:,3));

%% ============================================================
% BASIC TASK COMPLETION NUMBERS
% ============================================================

real_final_pos_err_vicon = norm(q_real_vicon(end,1:2) - goal(1:2));
real_final_pos_err_ekf   = norm(q_real_est(end,1:2)   - goal(1:2));

sim_final_pos_err_actual = norm(q_sim_actual(end,1:2) - goal(1:2));
sim_final_pos_err_ekf    = norm(q_sim_est(end,1:2)    - goal(1:2));

real_final_theta_err_vicon = abs(wrapToPiLocal(goal(3) - q_real_vicon(end,3)));
real_final_theta_err_ekf   = abs(wrapToPiLocal(goal(3) - q_real_est(end,3)));

sim_final_theta_err_actual = abs(wrapToPiLocal(goal(3) - q_sim_actual(end,3)));
sim_final_theta_err_ekf    = abs(wrapToPiLocal(goal(3) - q_sim_est(end,3)));

%% ============================================================
% LOCALIZATION ERRORS
% ============================================================

e_real_loc_xy = q_real_est(:,1:2) - q_real_vicon(:,1:2);
e_real_loc_pos = vecnorm(e_real_loc_xy, 2, 2);
e_real_loc_th = wrapToPiLocal(q_real_est(:,3) - q_real_vicon(:,3));

e_sim_loc_xy = q_sim_est(:,1:2) - q_sim_actual(:,1:2);
e_sim_loc_pos = vecnorm(e_sim_loc_xy, 2, 2);
e_sim_loc_th = wrapToPiLocal(q_sim_est(:,3) - q_sim_actual(:,3));

real_loc_rmse_pos = sqrt(mean(e_real_loc_pos.^2));
real_loc_max_pos  = max(e_real_loc_pos);
real_loc_final_pos = e_real_loc_pos(end);
real_loc_rmse_th = sqrt(mean(e_real_loc_th.^2));
real_loc_max_th  = max(abs(e_real_loc_th));

sim_loc_rmse_pos = sqrt(mean(e_sim_loc_pos.^2));
sim_loc_max_pos  = max(e_sim_loc_pos);
sim_loc_final_pos = e_sim_loc_pos(end);
sim_loc_rmse_th = sqrt(mean(e_sim_loc_th.^2));
sim_loc_max_th  = max(abs(e_sim_loc_th));

%% ============================================================
% RED AREA LOCALIZATION ERRORS
% ============================================================

in_red_real = q_real_vicon(:,1) >= red_x_min & q_real_vicon(:,1) <= red_x_max & ...
              q_real_vicon(:,2) >= red_y_min & q_real_vicon(:,2) <= red_y_max;

in_red_sim = q_sim_actual(:,1) >= red_x_min & q_sim_actual(:,1) <= red_x_max & ...
             q_sim_actual(:,2) >= red_y_min & q_sim_actual(:,2) <= red_y_max;

[real_red_t_start, real_red_t_end, real_red_duration] = redInterval(t_real, in_red_real);
[sim_red_t_start,  sim_red_t_end,  sim_red_duration]  = redInterval(t_sim,  in_red_sim);

real_red_rmse_pos = nan;
real_red_max_pos  = nan;
real_red_rmse_th  = nan;

if any(in_red_real)
    real_red_rmse_pos = sqrt(mean(e_real_loc_pos(in_red_real).^2));
    real_red_max_pos  = max(e_real_loc_pos(in_red_real));
    real_red_rmse_th  = sqrt(mean(e_real_loc_th(in_red_real).^2));
end

sim_red_rmse_pos = nan;
sim_red_max_pos  = nan;
sim_red_rmse_th  = nan;

if any(in_red_sim)
    sim_red_rmse_pos = sqrt(mean(e_sim_loc_pos(in_red_sim).^2));
    sim_red_max_pos  = max(e_sim_loc_pos(in_red_sim));
    sim_red_rmse_th  = sqrt(mean(e_sim_loc_th(in_red_sim).^2));
end

%% ============================================================
% CONTROL INPUTS AND WHEEL SPEEDS
% ============================================================

real_v_max_abs = max(abs(v_real), [], 'omitnan');
real_omega_max_abs = max(abs(omega_real), [], 'omitnan');

sim_v_max_abs = max(abs(v_sim), [], 'omitnan');
sim_omega_max_abs = max(abs(omega_sim), [], 'omitnan');

real_cmd_wheel_max_abs = max(abs(omega_wheels_real_cmd), [], 'all', 'omitnan');
real_meas_wheel_max_abs = max(abs(omega_wheels_real_meas), [], 'all', 'omitnan');
sim_cmd_wheel_max_abs = max(abs(omega_wheels_sim_cmd), [], 'all', 'omitnan');

real_cmd_wheel_min = min(omega_wheels_real_cmd, [], 1, 'omitnan');
real_cmd_wheel_max = max(omega_wheels_real_cmd, [], 1, 'omitnan');

real_meas_wheel_min = min(omega_wheels_real_meas, [], 1, 'omitnan');
real_meas_wheel_max = max(omega_wheels_real_meas, [], 1, 'omitnan');

sim_cmd_wheel_min = min(omega_wheels_sim_cmd, [], 1, 'omitnan');
sim_cmd_wheel_max = max(omega_wheels_sim_cmd, [], 1, 'omitnan');

%% ============================================================
% CLEARANCE / COLLISION CHECK
% ============================================================

real_min_obs_clearance = minObstacleClearance(q_real_vicon(:,1:2), obstacles);
sim_min_obs_clearance  = minObstacleClearance(q_sim_actual(:,1:2), obstacles);

real_min_wall_margin = minWallMargin(q_real_vicon(:,1:2), map_x_min, map_x_max, map_y_min, map_y_max);
sim_min_wall_margin  = minWallMargin(q_sim_actual(:,1:2), map_x_min, map_x_max, map_y_min, map_y_max);

%% ============================================================
% TRACKING ERROR TO CURRENT WAYPOINT REFERENCE
% ============================================================

e_real_track_pos = vecnorm(q_real_vicon(:,1:2) - q_real_target(:,1:2), 2, 2);
e_sim_track_pos  = vecnorm(q_sim_actual(:,1:2) - q_sim_target(:,1:2), 2, 2);

real_track_final = e_real_track_pos(end);
sim_track_final  = e_sim_track_pos(end);

real_track_rmse = sqrt(mean(e_real_track_pos.^2));
sim_track_rmse  = sqrt(mean(e_sim_track_pos.^2));

%% ============================================================
% PRINT REPORT NUMBERS
% ============================================================

fprintf('\n============================================================\n');
fprintf('TASK 2 REPORT NUMBERS\n');
fprintf('============================================================\n');

fprintf('\n--- Duration ---\n');
fprintf('Real duration:       %.3f s\n', t_real(end));
fprintf('Simulation duration: %.3f s\n', t_sim(end));

fprintf('\n--- Target sequence ---\n');
fprintf('Real unique q_target rows:\n');
disp(unique(round(q_real_target,3),'rows'));
fprintf('Simulation unique q_target rows:\n');
disp(unique(round(q_sim_target,3),'rows'));

fprintf('\n--- Start / final poses ---\n');
fprintf('Real start Vicon:       [%.4f %.4f %.4f]\n', q_real_vicon(1,1), q_real_vicon(1,2), q_real_vicon(1,3));
fprintf('Real final Vicon:       [%.4f %.4f %.4f]\n', q_real_vicon(end,1), q_real_vicon(end,2), q_real_vicon(end,3));
fprintf('Real final EKF:         [%.4f %.4f %.4f]\n', q_real_est(end,1), q_real_est(end,2), q_real_est(end,3));
fprintf('Simulation final actual:[%.4f %.4f %.4f]\n', q_sim_actual(end,1), q_sim_actual(end,2), q_sim_actual(end,3));
fprintf('Simulation final EKF:   [%.4f %.4f %.4f]\n', q_sim_est(end,1), q_sim_est(end,2), q_sim_est(end,3));

fprintf('\n--- Final errors to Task 2 goal [1.00, -0.30] ---\n');
fprintf('Real Vicon final position error:       %.4f m\n', real_final_pos_err_vicon);
fprintf('Real EKF final position error:         %.4f m\n', real_final_pos_err_ekf);
fprintf('Simulation actual final position error:%.4f m\n', sim_final_pos_err_actual);
fprintf('Simulation EKF final position error:   %.4f m\n', sim_final_pos_err_ekf);

fprintf('Real Vicon final theta error:          %.4f rad = %.2f deg\n', real_final_theta_err_vicon, rad2deg(real_final_theta_err_vicon));
fprintf('Real EKF final theta error:            %.4f rad = %.2f deg\n', real_final_theta_err_ekf, rad2deg(real_final_theta_err_ekf));
fprintf('Simulation actual final theta error:   %.4f rad = %.2f deg\n', sim_final_theta_err_actual, rad2deg(sim_final_theta_err_actual));
fprintf('Simulation EKF final theta error:      %.4f rad = %.2f deg\n', sim_final_theta_err_ekf, rad2deg(sim_final_theta_err_ekf));

fprintf('\n--- Localization error: EKF vs Vicon/actual ---\n');
fprintf('Real localization RMSE position:       %.4f m\n', real_loc_rmse_pos);
fprintf('Real localization max position error:  %.4f m\n', real_loc_max_pos);
fprintf('Real localization final position error:%.4f m\n', real_loc_final_pos);
fprintf('Real localization RMSE theta:          %.4f rad = %.2f deg\n', real_loc_rmse_th, rad2deg(real_loc_rmse_th));
fprintf('Real localization max theta error:     %.4f rad = %.2f deg\n', real_loc_max_th, rad2deg(real_loc_max_th));

fprintf('Simulation localization RMSE position:       %.4f m\n', sim_loc_rmse_pos);
fprintf('Simulation localization max position error:  %.4f m\n', sim_loc_max_pos);
fprintf('Simulation localization final position error:%.4f m\n', sim_loc_final_pos);
fprintf('Simulation localization RMSE theta:          %.4f rad = %.2f deg\n', sim_loc_rmse_th, rad2deg(sim_loc_rmse_th));
fprintf('Simulation localization max theta error:     %.4f rad = %.2f deg\n', sim_loc_max_th, rad2deg(sim_loc_max_th));

fprintf('\n--- Red-area localization ---\n');
fprintf('Real red-area time interval:       %.3f to %.3f s, duration %.3f s\n', real_red_t_start, real_red_t_end, real_red_duration);
fprintf('Real red-area RMSE position:       %.4f m\n', real_red_rmse_pos);
fprintf('Real red-area max position error:  %.4f m\n', real_red_max_pos);
fprintf('Real red-area RMSE theta:          %.4f rad = %.2f deg\n', real_red_rmse_th, rad2deg(real_red_rmse_th));

fprintf('Simulation red-area time interval: %.3f to %.3f s, duration %.3f s\n', sim_red_t_start, sim_red_t_end, sim_red_duration);
fprintf('Simulation red-area RMSE position: %.4f m\n', sim_red_rmse_pos);
fprintf('Simulation red-area max position error:%.4f m\n', sim_red_max_pos);
fprintf('Simulation red-area RMSE theta:    %.4f rad = %.2f deg\n', sim_red_rmse_th, rad2deg(sim_red_rmse_th));

fprintf('\n--- Control inputs ---\n');
fprintf('Real max |v|:       %.4f m/s\n', real_v_max_abs);
fprintf('Real max |omega|:   %.4f rad/s\n', real_omega_max_abs);
fprintf('Simulation max |v|: %.4f m/s\n', sim_v_max_abs);
fprintf('Simulation max |omega|: %.4f rad/s\n', sim_omega_max_abs);

fprintf('\n--- Wheel speeds ---\n');
fprintf('Wheel speed limit: %.2f rad/s\n', omega_limit);
fprintf('Real commanded wheel max |omega_wheel|: %.4f rad/s\n', real_cmd_wheel_max_abs);
fprintf('Real measured wheel max |omega_wheel|:  %.4f rad/s\n', real_meas_wheel_max_abs);
fprintf('Simulation command wheel max |omega_wheel|: %.4f rad/s\n', sim_cmd_wheel_max_abs);

fprintf('Real commanded wheel range L/R:\n');
fprintf('  left:  [%.4f, %.4f] rad/s\n', real_cmd_wheel_min(1), real_cmd_wheel_max(1));
fprintf('  right: [%.4f, %.4f] rad/s\n', real_cmd_wheel_min(2), real_cmd_wheel_max(2));

fprintf('Real measured wheel range L/R:\n');
fprintf('  left:  [%.4f, %.4f] rad/s\n', real_meas_wheel_min(1), real_meas_wheel_max(1));
fprintf('  right: [%.4f, %.4f] rad/s\n', real_meas_wheel_min(2), real_meas_wheel_max(2));

fprintf('Simulation commanded wheel range L/R:\n');
fprintf('  left:  [%.4f, %.4f] rad/s\n', sim_cmd_wheel_min(1), sim_cmd_wheel_max(1));
fprintf('  right: [%.4f, %.4f] rad/s\n', sim_cmd_wheel_min(2), sim_cmd_wheel_max(2));

fprintf('\n--- Collision / clearance check ---\n');
fprintf('Real minimum obstacle clearance:       %.4f m\n', real_min_obs_clearance);
fprintf('Simulation minimum obstacle clearance: %.4f m\n', sim_min_obs_clearance);
fprintf('Real minimum wall margin:              %.4f m\n', real_min_wall_margin);
fprintf('Simulation minimum wall margin:        %.4f m\n', sim_min_wall_margin);

fprintf('\n--- Tracking error to current waypoint reference ---\n');
fprintf('Real final tracking position error:       %.4f m\n', real_track_final);
fprintf('Real tracking RMSE position:              %.4f m\n', real_track_rmse);
fprintf('Simulation final tracking position error: %.4f m\n', sim_track_final);
fprintf('Simulation tracking RMSE position:        %.4f m\n', sim_track_rmse);

fprintf('\n============================================================\n');

%% ============================================================
% Helper functions
% ============================================================

function X = forceCols(X, ncols)
    X = squeeze(X);

    if isvector(X)
        X = X(:).';
    end

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Expected %d columns, got size %dx%d.', ncols, size(X,1), size(X,2));
    end
end

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end

function [tStart, tEnd, duration] = redInterval(t, mask)
    if any(mask)
        idx = find(mask);
        tStart = t(idx(1));
        tEnd = t(idx(end));
        duration = tEnd - tStart;
    else
        tStart = NaN;
        tEnd = NaN;
        duration = 0;
    end
end

function minClear = minObstacleClearance(P, obstacles)
    dmin = inf(size(P,1),1);

    for k = 1:size(obstacles,1)
        rect = obstacles(k,:);
        d = pointRectDistance(P, rect);
        dmin = min(dmin, d);
    end

    minClear = min(dmin);
end

function d = pointRectDistance(P, rect)
    x = P(:,1);
    y = P(:,2);

    xmin = rect(1);
    ymin = rect(2);
    xmax = rect(1) + rect(3);
    ymax = rect(2) + rect(4);

    dx = max(max(xmin - x, 0), x - xmax);
    dy = max(max(ymin - y, 0), y - ymax);

    d = sqrt(dx.^2 + dy.^2);
end

function margin = minWallMargin(P, xmin, xmax, ymin, ymax)
    distLeft   = P(:,1) - xmin;
    distRight  = xmax - P(:,1);
    distBottom = P(:,2) - ymin;
    distTop    = ymax - P(:,2);

    margin = min([distLeft; distRight; distBottom; distTop]);
end


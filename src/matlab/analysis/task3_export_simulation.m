clear; clc; close all;

%% ============================================================
% Robust export script for Task 3 simulation
%
% Output:
%   exported_task3_simulation.mat
%
% This script:
%   1. Sets the Task 3 parameters in the base workspace.
%   2. Runs the Task 3 Simulink model.
%   3. Extracts useful outputs even when To Workspace blocks save arrays,
%      timeseries, structures, or SimulationData.Signal objects.
%   4. Crops the circular-tracking interval.
%   5. Saves a clean Dsim_task3 structure for plotting/reporting.
% ============================================================

%% ---------------- SCRIPT FOLDER ----------------
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    scriptFolder = pwd;
else
    scriptFolder = fileparts(scriptPath);
end

cd(scriptFolder);

fprintf('\nScript folder:\n%s\n', scriptFolder);

%% ---------------- FILES ----------------
% Put your Task 3 model name first. If you kept the old name, the script
% will automatically use the first model that exists in this folder.
modelCandidates = { ...
    'task3_model', ...
    'task3_simulation', ...
    'task2_simulation', ...
    'getViconMeasure_2024b'};

modelName = '';
for i = 1:numel(modelCandidates)
    if isfile(fullfile(scriptFolder, [modelCandidates{i} '.slx']))
        modelName = modelCandidates{i};
        break;
    end
end

if isempty(modelName)
    error('No Simulink model found. Tried: %s', strjoin(modelCandidates, ', '));
end

modelFile = fullfile(scriptFolder, [modelName '.slx']);
saveFile  = fullfile(scriptFolder, 'exported_task3_simulation_from_task2.mat');
plotsFolder = fullfile(scriptFolder, 'plots_task3');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

fprintf('\nUsing model:\n%s\n', modelFile);
fprintf('Output file:\n%s\n', saveFile);

%% ---------------- PARAMETERS ----------------
% Mode
is_real_robot = 0;

% Sampling and robot parameters
Ts = 0.04;
r  = 0.03;
d  = 0.165;
omega_M = 10;       % wheel speed limit [rad/s]

% Controller gains used by Controller_Task3
% Start with the previous values. If tracking is slow/oscillatory, tune later.
k_x     = 0.5;
k_y     = 1.2;
k_theta = 0.3;

% Controller saturation parameters, if the model uses them from workspace
v_max     = 0.22;
omega_max = 0.60;

% Map coordinates from Final Activity
x1 = -2.90;
x2 = -1.60;
x3 = -1.50;
x4 = -1.40;
x5 = -0.60;
x6 =  0.20;
x7 =  1.00;
x8 =  1.45;
x9 =  1.80;
x10 = 2.20;
x11 = 2.90;

y1 = -1.50;
y2 = -1.10;
y3 = -0.70;
y4 = -0.30;
y5 = -0.20;
y6 =  0.10;
y7 =  0.20;
y8 =  1.50;

x_coords = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11];
y_coords = [y1 y2 y3 y4 y5 y6 y7 y8];

% Previous task goals, kept for model compatibility
q_goal_task1 = [x2; y2; 0];
q_goal_task2 = [x7; y4; 0];
q_goal2 = q_goal_task2;
q_wp1 = [-0.20; -1.10; 0];
q_wp2 = [-0.20; -0.30; pi/2];

% Task 3 circular trajectory parameters
p_start = [x8; y4];
circle_center = [2.00; 0.00];
R_circle = norm(p_start - circle_center);
circle_dir = +1;

% IMPORTANT: this must match the hard-coded omega in your Task_Manager.
% In the Task_Manager code shown in sim.pdf, omega_circle_abs = 0.30.
omega_circle = 0.30;
N_turns = 1.5;
T_circle = 2*pi*N_turns/omega_circle;
alpha0 = atan2(p_start(2) - circle_center(2), p_start(1) - circle_center(1));
theta_start3 = wrapToPiLocal(alpha0 + circle_dir*pi/2);
q_start_task3 = [p_start; theta_start3];

% For Task 3-only simulation, start directly at the required circular start.
q_goal_task2 = [1.00; -0.30; 0.00];
q_init = q_goal_task2;

% Fake Vicon / localization parameters
sigma_e = 0.02;
p_loss = 0.90;

% Simulation stop time: T_circle plus a margin.
T_transfer_est = 10.0;
T_stop_buffer  = 4.0;
simStopTime = T_transfer_est + T_circle + T_stop_buffer;

% Cropping settings
cropMarginBefore = 0.00;
cropMarginAfter  = 0.20;

%% ---------------- PUSH VARIABLES TO BASE WORKSPACE ----------------
varsToPush = { ...
    'is_real_robot', 'Ts', 'r', 'd', 'omega_M', ...
    'k_x', 'k_y', 'k_theta', 'v_max', 'omega_max', ...
    'x_coords', 'y_coords', ...
    'q_goal_task1', 'q_goal_task2', 'q_goal2', 'q_wp1', 'q_wp2', ...
    'q_init', 'q_start_task3', 'circle_center', 'R_circle', ...
    'circle_dir', 'omega_circle', 'N_turns', 'T_circle', ...
    'T_transfer_est', 'T_stop_buffer', ...
    'alpha0', 'theta_start3', 'sigma_e', 'p_loss'};

for i = 1:numel(varsToPush)
    assignin('base', varsToPush{i}, eval(varsToPush{i}));
end

fprintf('\nTask 3 parameters:\n');
fprintf('q_init = [%.4f %.4f %.4f]^T\n', q_init(1), q_init(2), q_init(3));
fprintf('circle_center = [%.4f %.4f]^T\n', circle_center(1), circle_center(2));
fprintf('R_circle = %.4f m\n', R_circle);
fprintf('omega_circle = %.4f rad/s\n', omega_circle);
fprintf('T_circle = %.4f s\n', T_circle);
fprintf('simStopTime = %.4f s\n', simStopTime);
fprintf('is_real_robot = %d\n', is_real_robot);

%% ---------------- RUN SIMULATION ----------------
fprintf('\nRunning model: %s\n', modelName);

load_system(modelName);

% Visualizer is intentionally kept ON.
% If the simulation becomes slow, reduce drawing frequency inside the
% Visualizer MATLAB Function block instead of commenting it out here.
fprintf('Visualizer is kept active.\n');

try
    set_param(modelName, 'FastRestart', 'off');
catch
end

try
    set_param(modelName, 'SimulationMode', 'normal');
catch
end

set_param(modelName, 'StopTime', num2str(simStopTime));

% Clear likely old variables from base workspace to avoid stale extraction.
evalin('base', ['clear out q_est q_actual q_true q_target v omega omega_wheels ', ...
                'u_actual q_est_imu Y_vicon_used Y_vicon_use Y_viscon ', ...
                'Y_vicon_turtlebot Y_vicon_ground_truth Y_vicon_ground_truth_sim ', ...
                'vicon_valid loss_flag in_red in_orange state_out task3_active ', ...
                'task3_done elapsed_circle t tout']);

out = sim(modelName, 'ReturnWorkspaceOutputs', 'on');

fprintf('\nSimulation finished.\n');

%% ---------------- DISPLAY AVAILABLE OUTPUTS ----------------
try
    outNames = out.who;
    fprintf('\nSimulation output names inside SimulationOutput:\n');
    disp(outNames.');
catch
    fprintf('\nCould not display SimulationOutput names.\n');
end

fprintf('\nRelevant variables currently visible in base workspace:\n');
try
    evalin('base', "whos q_est q_actual q_true q_target v omega omega_wheels u_actual Y_vicon* Y_viscon* state_out task3_active elapsed_circle loss_flag in_red in_orange");
catch
end

%% ---------------- EXTRACT REQUIRED SIGNALS ----------------
% The names list includes multiple spellings because previous models used
% both vicon/viscon and q_actual/q_true.
q_est    = getRequiredPose(out, {'q_est'});
q_actual = getRequiredPose(out, {'q_actual', 'q_true', 'q'});
q_target = getRequiredPose(out, {'q_target'});

v            = getRequiredScalar(out, {'v'});
omega        = getRequiredScalar(out, {'omega'});
omega_wheels = getRequiredMatrix(out, {'omega_wheels', 'bot_wheelspeed'}, 2);

Nbase = min([size(q_est,1), size(q_actual,1), size(q_target,1), ...
             size(v,1), size(omega,1), size(omega_wheels,1)]);

%% ---------------- OPTIONAL SIGNALS ----------------
q_est_imu = getOptionalMatrix(out, {'q_est_imu'}, 3, Nbase);
u_actual  = getOptionalMatrix(out, {'u_actual'}, 2, Nbase);

Y_vicon_used = getOptionalPoseWithFallback(out, ...
    {'Y_vicon_used', 'Y_vicon_use', 'Y_vicon_turtlebot', ...
     'Y_viscon_used', 'Y_viscon'}, q_actual);

Y_vicon_ground_truth_sim = getOptionalPoseWithFallback(out, ...
    {'Y_vicon_ground_truth_sim', 'Y_vicon_ground_truth', ...
     'Y_viscon_ground_truth', 'Y_viscon_ground_trith', 'vicon_ground'}, q_actual);

vicon_valid = getOptionalScalar(out, {'vicon_valid'}, Nbase);
loss_flag   = getOptionalScalar(out, {'loss_flag'}, Nbase);
in_red      = getOptionalScalar(out, {'in_red'}, Nbase);
in_orange   = getOptionalScalar(out, {'in_orange'}, Nbase);
state_out   = getOptionalScalar(out, {'state_out'}, Nbase);
task3_active = getOptionalScalar(out, {'task3_active'}, Nbase);
task3_done   = getOptionalScalar(out, {'task3_done'}, Nbase);
elapsed_circle = getOptionalScalar(out, {'elapsed_circle'}, Nbase);

%% ---------------- TIME ----------------
t = getTimeVector(out, Nbase, Ts);

%% ---------------- ALIGN LENGTHS ----------------
N = min([
    size(q_est,1)
    size(q_actual,1)
    size(q_target,1)
    size(v,1)
    size(omega,1)
    size(omega_wheels,1)
    size(q_est_imu,1)
    size(u_actual,1)
    size(Y_vicon_used,1)
    size(Y_vicon_ground_truth_sim,1)
    size(vicon_valid,1)
    size(loss_flag,1)
    size(in_red,1)
    size(in_orange,1)
    size(state_out,1)
    size(task3_active,1)
    size(task3_done,1)
    size(elapsed_circle,1)
    size(t,1)
]);

q_est = q_est(1:N,:);
q_actual = q_actual(1:N,:);
q_target = q_target(1:N,:);
v = v(1:N,:);
omega = omega(1:N,:);
omega_wheels = omega_wheels(1:N,:);
q_est_imu = q_est_imu(1:N,:);
u_actual = u_actual(1:N,:);
Y_vicon_used = Y_vicon_used(1:N,:);
Y_vicon_ground_truth_sim = Y_vicon_ground_truth_sim(1:N,:);
vicon_valid = vicon_valid(1:N,:);
loss_flag = loss_flag(1:N,:);
in_red = in_red(1:N,:);
in_orange = in_orange(1:N,:);
state_out = state_out(1:N,:);
task3_active = task3_active(1:N,:);
task3_done = task3_done(1:N,:);
elapsed_circle = elapsed_circle(1:N,:);
t = t(1:N,:);

t = t(:) - t(1);

% Wrap all pose angles
q_est(:,3) = wrapToPiLocal(q_est(:,3));
q_actual(:,3) = wrapToPiLocal(q_actual(:,3));
q_target(:,3) = wrapToPiLocal(q_target(:,3));
q_est_imu(:,3) = wrapToPiLocal(q_est_imu(:,3));
Y_vicon_used(:,3) = wrapToPiLocal(Y_vicon_used(:,3));
Y_vicon_ground_truth_sim(:,3) = wrapToPiLocal(Y_vicon_ground_truth_sim(:,3));

fprintf('\nFirst q_target:\n'); disp(q_target(1,:).');
fprintf('Middle q_target:\n'); disp(q_target(round(N/2),:).');
fprintf('Final q_target before cropping:\n'); disp(q_target(end,:).');

%% ============================================================
% ROBUST TASK 3 CROPPING
% ============================================================
%% ============================================================
% FULL SEGMENT CROPPING
% ============================================================
% For comparison with the real robot run, keep the full segment:
% Task 2 end -> transfer to Task 3 start -> circle -> stop.
%
% The circle-only interval is still stored later using task3_active/state_out.

idxStart = 1;
idxEnd = N;
idx = idxStart:idxEnd;

cropReason = 'full segment from Task 2 end';

fprintf('\nCropping summary:\n');
fprintf('Reason   = %s\n', cropReason);
fprintf('idxStart = %d, tStart = %.3f s\n', idxStart, t(idxStart));
fprintf('idxEnd   = %d, tEnd   = %.3f s\n', idxEnd, t(idxEnd));
fprintf('Exported samples = %d\n', numel(idx));
%% ---------------- BUILD EXPORT STRUCT ----------------
Dsim_task3 = struct();

Dsim_task3.q_est = q_est(idx,:);
Dsim_task3.q_actual = q_actual(idx,:);
Dsim_task3.q_target = q_target(idx,:);

Dsim_task3.v = v(idx,:);
Dsim_task3.omega = omega(idx,:);
Dsim_task3.omega_wheels = omega_wheels(idx,:);

Dsim_task3.q_est_imu = q_est_imu(idx,:);
Dsim_task3.u_actual = u_actual(idx,:);

% Compatibility aliases
Dsim_task3.bot_wheelspeed = omega_wheels(idx,:);
Dsim_task3.Y_vicon_used = Y_vicon_used(idx,:);
Dsim_task3.Y_vicon_ground_truth_sim = Y_vicon_ground_truth_sim(idx,:);
Dsim_task3.Y_vicon_turtlebot = Y_vicon_used(idx,:);
Dsim_task3.Y_vicon_ground_truth = Y_vicon_ground_truth_sim(idx,:);

Dsim_task3.vicon_valid = vicon_valid(idx,:);
Dsim_task3.loss_flag = loss_flag(idx,:);
Dsim_task3.in_red = in_red(idx,:);
Dsim_task3.in_orange = in_orange(idx,:);
Dsim_task3.state_out = state_out(idx,:);
Dsim_task3.task3_active = task3_active(idx,:);
Dsim_task3.task3_done = task3_done(idx,:);
Dsim_task3.elapsed_circle = elapsed_circle(idx,:);

Dsim_task3.t = t(idx,:);
Dsim_task3.t = Dsim_task3.t(:);
Dsim_task3.t = Dsim_task3.t - Dsim_task3.t(1);

Dsim_task3.q_init = q_init;
Dsim_task3.q_start_task3 = q_start_task3;
Dsim_task3.circle_center = circle_center;
Dsim_task3.R_circle = R_circle;
Dsim_task3.circle_dir = circle_dir;
Dsim_task3.omega_circle = omega_circle;
Dsim_task3.N_turns = N_turns;
Dsim_task3.T_circle = T_circle;
Dsim_task3.alpha0 = alpha0;
Dsim_task3.theta_start3 = theta_start3;
Dsim_task3.q_goal_task2 = q_goal_task2;
Dsim_task3.T_transfer_est = T_transfer_est;
Dsim_task3.T_stop_buffer = T_stop_buffer;


Dsim_task3.Ts = Ts;
Dsim_task3.r = r;
Dsim_task3.d = d;
Dsim_task3.k_x = k_x;
Dsim_task3.k_y = k_y;
Dsim_task3.k_theta = k_theta;
Dsim_task3.v_max = v_max;
Dsim_task3.omega_max = omega_max;
Dsim_task3.omega_M = omega_M;
Dsim_task3.sigma_e = sigma_e;
Dsim_task3.p_loss = p_loss;

Dsim_task3.idx_start_original = idxStart;
Dsim_task3.idx_end_original = idxEnd;
Dsim_task3.cropReason = cropReason;
Dsim_task3.source_model = modelName;
%% ---------------- SIMULATION PHASE MARKERS ----------------
% Detect the beginning of the circular tracking phase.
% Preferred method: task3_active from Task_Manager.

idxSimCircle = [];

if anyValid(Dsim_task3.task3_active)
    idxSimCircle = find(Dsim_task3.task3_active > 0.5, 1, 'first');
end

% Fallback: state_out == 2
if isempty(idxSimCircle) && anyValid(Dsim_task3.state_out)
    idxSimCircle = find(round(Dsim_task3.state_out) == 2, 1, 'first');
end

% Fallback: moving circular q_target
if isempty(idxSimCircle)
    dTarget = [0; vecnorm(diff(Dsim_task3.q_target(:,1:2),1,1), 2, 2)];
    idxSimCircle = find(dTarget > 1e-5, 1, 'first');
end

if isempty(idxSimCircle)
    idxSimCircle = 1;
end

% Detect the end of commanded motion.
motionMaskSim = abs(Dsim_task3.v) > 0.01 | abs(Dsim_task3.omega) > 0.03;

if any(motionMaskSim)
    idxSimStop = find(motionMaskSim, 1, 'last');
else
    idxSimStop = numel(Dsim_task3.t);
end

Dsim_task3.idx_circle_start = idxSimCircle;
Dsim_task3.t_circle_start = Dsim_task3.t(idxSimCircle);

Dsim_task3.idx_motion_end = idxSimStop;
Dsim_task3.t_motion_end = Dsim_task3.t(idxSimStop);

fprintf('\nSimulation phase markers:\n');
fprintf('idx_circle_start = %d, t_circle_start = %.3f s\n', ...
    Dsim_task3.idx_circle_start, Dsim_task3.t_circle_start);
fprintf('idx_motion_end   = %d, t_motion_end   = %.3f s\n', ...
    Dsim_task3.idx_motion_end, Dsim_task3.t_motion_end);

%% ---------------- METRICS ----------------
tracking_pos_err = vecnorm(Dsim_task3.q_actual(:,1:2) - Dsim_task3.q_target(:,1:2), 2, 2);
tracking_theta_err = abs(wrapToPiLocal(Dsim_task3.q_target(:,3) - Dsim_task3.q_actual(:,3)));

loc_pos_err = vecnorm(Dsim_task3.q_est(:,1:2) - Dsim_task3.q_actual(:,1:2), 2, 2);
loc_theta_err = abs(wrapToPiLocal(Dsim_task3.q_est(:,3) - Dsim_task3.q_actual(:,3)));

validLoss = Dsim_task3.loss_flag(~isnan(Dsim_task3.loss_flag));
if isempty(validLoss)
    vicon_loss_percentage = NaN;
else
    vicon_loss_percentage = 100 * mean(validLoss > 0.5);
end

% Clearance from the nominal cube rectangle [x9,x10] x [y6,y7].
% The exact physical robot footprint is not subtracted here; this is path-point clearance.
cubeRect = [x9, x10, y6, y7];
clearance_to_cube = pointRectClearance(Dsim_task3.q_actual(:,1:2), cubeRect);

Dsim_task3.metrics = struct();
Dsim_task3.metrics.duration = Dsim_task3.t(end) - Dsim_task3.t(1);
Dsim_task3.metrics.tracking_pos_rmse = sqrt(mean(tracking_pos_err.^2));
Dsim_task3.metrics.tracking_pos_max = max(tracking_pos_err);
Dsim_task3.metrics.tracking_pos_final = tracking_pos_err(end);
Dsim_task3.metrics.tracking_theta_rmse = sqrt(mean(tracking_theta_err.^2));
Dsim_task3.metrics.tracking_theta_max = max(tracking_theta_err);
Dsim_task3.metrics.tracking_theta_final = tracking_theta_err(end);
Dsim_task3.metrics.localization_pos_rmse = sqrt(mean(loc_pos_err.^2));
Dsim_task3.metrics.localization_pos_max = max(loc_pos_err);
Dsim_task3.metrics.localization_theta_rmse = sqrt(mean(loc_theta_err.^2));
Dsim_task3.metrics.localization_theta_max = max(loc_theta_err);
Dsim_task3.metrics.vicon_loss_percentage = vicon_loss_percentage;
Dsim_task3.metrics.max_abs_v = max(abs(Dsim_task3.v));
Dsim_task3.metrics.max_abs_omega = max(abs(Dsim_task3.omega));
Dsim_task3.metrics.max_abs_wheel_speed = max(abs(Dsim_task3.omega_wheels), [], 'all');
Dsim_task3.metrics.min_clearance_to_cube = min(clearance_to_cube);

fprintf('\n============================================================\n');
fprintf('EXPORTED TASK 3 SIMULATION CHECK\n');
fprintf('============================================================\n');
fprintf('Exported duration = %.3f s\n', Dsim_task3.metrics.duration);
fprintf('Tracking position RMSE = %.4f m\n', Dsim_task3.metrics.tracking_pos_rmse);
fprintf('Tracking position max  = %.4f m\n', Dsim_task3.metrics.tracking_pos_max);
fprintf('Final tracking position error = %.4f m\n', Dsim_task3.metrics.tracking_pos_final);
fprintf('Tracking theta RMSE = %.4f rad = %.2f deg\n', ...
    Dsim_task3.metrics.tracking_theta_rmse, rad2deg(Dsim_task3.metrics.tracking_theta_rmse));
fprintf('Final tracking theta error = %.4f rad = %.2f deg\n', ...
    Dsim_task3.metrics.tracking_theta_final, rad2deg(Dsim_task3.metrics.tracking_theta_final));
fprintf('Localization position RMSE = %.4f m\n', Dsim_task3.metrics.localization_pos_rmse);
fprintf('Localization position max  = %.4f m\n', Dsim_task3.metrics.localization_pos_max);
fprintf('Vicon loss percentage = %.2f %%\n', Dsim_task3.metrics.vicon_loss_percentage);
fprintf('Max |v| = %.4f m/s\n', Dsim_task3.metrics.max_abs_v);
fprintf('Max |omega| = %.4f rad/s\n', Dsim_task3.metrics.max_abs_omega);
fprintf('Max |wheel speed| = %.4f rad/s\n', Dsim_task3.metrics.max_abs_wheel_speed);
fprintf('Minimum point clearance to cube rectangle = %.4f m\n', Dsim_task3.metrics.min_clearance_to_cube);

fprintf('\nInitial exported q_actual:\n');
fprintf('[%.4f %.4f %.4f]\n', Dsim_task3.q_actual(1,1), Dsim_task3.q_actual(1,2), Dsim_task3.q_actual(1,3));
fprintf('Final exported q_actual:\n');
fprintf('[%.4f %.4f %.4f]\n', Dsim_task3.q_actual(end,1), Dsim_task3.q_actual(end,2), Dsim_task3.q_actual(end,3));

%% ---------------- SAVE ----------------
save(saveFile, 'Dsim_task3');

fprintf('\nSaved simulation data here:\n%s\n', saveFile);
fprintf('\nSaved fields in Dsim_task3:\n');
disp(fieldnames(Dsim_task3));

%% ---------------- QUICK CHECK PLOT ----------------
fig = figure('Color','w', 'Name','Task 3 simulation export check');
hold on; grid on; axis equal;

% Orange region
patch([x7 x11 x11 x7], [y1 y1 y8 y8], [1 0.7 0.3], ...
    'FaceAlpha', 0.15, 'EdgeColor', 'none');

% Red region
patch([x5 x6 x6 x5], [y1 y1 y6 y6], [1 0 0], ...
    'FaceAlpha', 0.10, 'EdgeColor', 'none');

% Cube rectangle
patch([x9 x10 x10 x9], [y6 y6 y7 y7], [0.2 0.2 0.2], ...
    'FaceAlpha', 0.25, 'EdgeColor', 'k', 'LineWidth', 1.0);

plot(Dsim_task3.q_target(:,1), Dsim_task3.q_target(:,2), ...
    'k--', 'LineWidth', 1.4);

plot(Dsim_task3.q_actual(:,1), Dsim_task3.q_actual(:,2), ...
    'g-.', 'LineWidth', 1.8);

plot(Dsim_task3.q_est(:,1), Dsim_task3.q_est(:,2), ...
    'Color', [0.8500 0.3250 0.0980], 'LineStyle','--', 'LineWidth', 1.5);

plot(Dsim_task3.q_actual(1,1), Dsim_task3.q_actual(1,2), ...
    'go', 'MarkerFaceColor','g', 'MarkerSize',8);

plot(Dsim_task3.q_actual(end,1), Dsim_task3.q_actual(end,2), ...
    'kx', 'MarkerSize',12, 'LineWidth',2);

plot(circle_center(1), circle_center(2), 'ko', 'MarkerFaceColor','w', 'MarkerSize',7);

xlabel('x [m]');
ylabel('y [m]');
title('Exported Task 3 simulation check');
legend('orange area','red area','cube','reference','simulation actual','localizer estimate', ...
    'start','end','circle center', 'Location','bestoutside');

xlim([0.6 3.0]);
ylim([-1.0 0.8]);

exportgraphics(fig, fullfile(plotsFolder, 'check_exported_task3_simulation.png'), ...
    'Resolution',300, 'BackgroundColor','white');
exportgraphics(fig, fullfile(plotsFolder, 'check_exported_task3_simulation.pdf'), ...
    'ContentType','vector', 'BackgroundColor','white');

fprintf('\nSaved quick-check plots:\n%s\n%s\n', ...
    fullfile(plotsFolder, 'check_exported_task3_simulation.png'), ...
    fullfile(plotsFolder, 'check_exported_task3_simulation.pdf'));

%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function q = getRequiredPose(out, names)
    q = getRequiredMatrix(out, names, 3);
    q(:,3) = wrapToPiLocal(q(:,3));
end

function x = getRequiredScalar(out, names)
    x = getRequiredMatrix(out, names, 1);
end

function x = getOptionalScalar(out, names, Ndefault)
    x = getOptionalMatrix(out, names, 1, Ndefault);
end

function X = getRequiredMatrix(out, names, ncols)
    for i = 1:numel(names)
        name = names{i};
        if hasSignal(out, name)
            raw = getRaw(getSignal(out, name));
            [ok, X] = tryForceCols(raw, ncols);
            if ok
                return;
            else
                fprintf('Signal "%s" exists but is not %d-column.\n', name, ncols);
            end
        end
    end
    error('None of the required signals were found with %d columns: %s', ...
        ncols, strjoin(names, ', '));
end

function X = getOptionalMatrix(out, names, ncols, Ndefault)
    for i = 1:numel(names)
        name = names{i};
        if hasSignal(out, name)
            raw = getRaw(getSignal(out, name));
            [ok, X] = tryForceCols(raw, ncols);
            if ok
                return;
            else
                fprintf('Optional signal "%s" exists but is not %d-column.\n', name, ncols);
            end
        end
    end
    X = NaN(Ndefault, ncols);
    fprintf('Optional signal not found. Filled with NaN. Names tried: %s\n', ...
        strjoin(names, ', '));
end

function Q = getOptionalPoseWithFallback(out, names, fallback)
    Ndefault = size(fallback,1);
    Q = getOptionalMatrix(out, names, 3, Ndefault);
    if all(isnan(Q(:)))
        Q = fallback;
    end
    Q(:,3) = wrapToPiLocal(Q(:,3));
end

function t = getTimeVector(out, Ndefault, Ts)
    if hasSignal(out, 't')
        raw = getRaw(getSignal(out, 't'));
        [ok, t] = tryForceCols(raw, 1);
        if ok
            t = t(:);
            return;
        end
    end

    if hasSignal(out, 'tout')
        raw = getRaw(getSignal(out, 'tout'));
        [ok, t] = tryForceCols(raw, 1);
        if ok
            t = t(:);
            return;
        end
    end

    try
        t = out.tout(:);
    catch
        t = (0:Ndefault-1).' * Ts;
    end
end

function tf = hasSignal(out, name)
    tf = false;

    % 1) Signal inside SimulationOutput
    try
        tf = any(strcmp(out.who, name));
    catch
        tf = false;
    end

    % 2) Variable directly in base workspace
    if ~tf
        try
            tf = evalin('base', sprintf('exist(''%s'', ''var'') == 1', name));
        catch
            tf = false;
        end
    end
end

function sig = getSignal(out, name)
    try
        if any(strcmp(out.who, name))
            sig = out.get(name);
            return;
        end
    catch
    end

    try
        sig = evalin('base', name);
        return;
    catch
    end

    error('Signal "%s" not found in SimulationOutput or base workspace.', name);
end

function X = getRaw(sig)
    if isa(sig, 'timeseries')
        X = squeeze(sig.Data);

    elseif isa(sig, 'Simulink.SimulationData.Signal')
        X = squeeze(sig.Values.Data);

    elseif isa(sig, 'Simulink.SimulationData.Dataset')
        if sig.numElements < 1
            error('Empty Dataset signal.');
        end
        elem = sig.get(1);
        X = getRaw(elem);

    elseif isnumeric(sig) || islogical(sig)
        X = squeeze(double(sig));

    elseif isstruct(sig) && isfield(sig, 'signals')
        X = squeeze(sig.signals.values);

    elseif isstruct(sig) && isfield(sig, 'Data')
        X = squeeze(sig.Data);

    elseif isstruct(sig) && isfield(sig, 'time') && isfield(sig, 'signals')
        X = squeeze(sig.signals.values);

    else
        error('Unsupported signal format: %s', class(sig));
    end
end

function X = forceCols(X, ncols)
    X = squeeze(X);

    if isvector(X)
        X = X(:);
    end

    % Convert dimensions such as 3 x N to N x 3.
    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    % Convert dimensions such as N x 1 x 3 or 1 x N x 3 after squeeze when possible.
    if ndims(X) > 2
        X = squeeze(X);
    end

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Expected %d columns, got size %dx%d.', ncols, size(X,1), size(X,2));
    end
end

function [ok, X] = tryForceCols(raw, ncols)
    ok = false;
    X = [];
    try
        Xtmp = forceCols(raw, ncols);
        ok = true;
        X = Xtmp;
    catch
        ok = false;
        X = [];
    end
end

function tf = anyValid(x)
    tf = ~isempty(x) && ~all(isnan(x(:)));
end

function clearance = pointRectClearance(P, rect)
    % Distance from points P=[x,y] to rectangle [xmin,xmax,ymin,ymax].
    xmin = rect(1); xmax = rect(2); ymin = rect(3); ymax = rect(4);
    px = P(:,1); py = P(:,2);

    dx = max([xmin - px, zeros(size(px)), px - xmax], [], 2);
    dy = max([ymin - py, zeros(size(py)), py - ymax], [], 2);
    outsideDist = sqrt(dx.^2 + dy.^2);

    inside = (px >= xmin) & (px <= xmax) & (py >= ymin) & (py <= ymax);
    insideDist = min([px-xmin, xmax-px, py-ymin, ymax-py], [], 2);

    clearance = outsideDist;
    clearance(inside) = -insideDist(inside);
end

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end
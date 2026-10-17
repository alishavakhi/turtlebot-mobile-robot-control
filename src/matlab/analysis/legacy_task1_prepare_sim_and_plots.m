clear; clc; close all;

%% ============================================================
% Task 1 — prepare simulation output and report plots
% Folder must contain:
%   test3_task1_all_outputs.mat
%   task1_simulation.slx
%
% This script:
% 1) loads real Task 1 data
% 2) runs simulation from the same real initial condition
% 3) exports simulation data
% 4) creates clean report plots in ./plots
% ============================================================

%% --------- FIND SCRIPT FOLDER ---------
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    scriptFolder = pwd;
else
    scriptFolder = fileparts(scriptPath);
end

fprintf('\nScript folder:\n%s\n', scriptFolder);

%% --------- FILES ---------
realFile = fullfile(scriptFolder, 'test3_task1_all_outputs.mat');
modelName = 'task1_simulation';
modelFile = fullfile(scriptFolder, [modelName '.slx']);

plotsFolder = fullfile(scriptFolder, 'plots');
if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

%% --------- LOAD REAL TASK 1 DATA ---------
if ~isfile(realFile)
    error('Real Task 1 file not found:\n%s', realFile);
end

S = load(realFile);

if isfield(S, 'D_task1')
    Dreal = S.D_task1;
else
    error('Variable D_task1 not found in real file.');
end

fprintf('\nLoaded real Task 1 data:\n%s\n', realFile);

%% --------- SELECT REFERENCE SIGNAL ---------
if isfield(Dreal, 'Y_viscon_ground_trith')
    q_ref_real = forceCols(Dreal.Y_viscon_ground_trith, 3);
    refName = 'Vicon ground truth';
elseif isfield(Dreal, 'Y_viscon')
    q_ref_real = forceCols(Dreal.Y_viscon, 3);
    refName = 'Vicon/ROS measurement';
else
    error('No Vicon/reference pose found in Dreal.');
end

q_est_real = forceCols(Dreal.q_est, 3);
q_imu_real = forceCols(Dreal.q_est_imu, 3);
q_target_real = forceCols(Dreal.q_target, 3);

t_real = Dreal.t(:);
v_real = Dreal.v(:);
omega_real = Dreal.omega(:);
omega_wheels_real = forceCols(Dreal.omega_wheels, 2);

if isfield(Dreal, 'bot_wheelspeed')
    bot_wheelspeed_real = forceCols(Dreal.bot_wheelspeed, 2);
else
    bot_wheelspeed_real = NaN(size(omega_wheels_real));
end

Nreal = min([length(t_real), size(q_ref_real,1), size(q_est_real,1), ...
             size(q_imu_real,1), size(q_target_real,1), ...
             length(v_real), length(omega_real), size(omega_wheels_real,1)]);

t_real = t_real(1:Nreal);
q_ref_real = q_ref_real(1:Nreal,:);
q_est_real = q_est_real(1:Nreal,:);
q_imu_real = q_imu_real(1:Nreal,:);
q_target_real = q_target_real(1:Nreal,:);
v_real = v_real(1:Nreal);
omega_real = omega_real(1:Nreal);
omega_wheels_real = omega_wheels_real(1:Nreal,:);
bot_wheelspeed_real = bot_wheelspeed_real(1:Nreal,:);

q_goal_task1 = q_target_real(1,:).';
q_init = q_ref_real(1,:).';

fprintf('\nReference used: %s\n', refName);
fprintf('q_init from real reference:\n');
disp(q_init);
fprintf('q_goal_task1:\n');
disp(q_goal_task1);

%% ============================================================
% RUN SIMULATION FROM REAL INITIAL CONDITION
% ============================================================

if ~isfile(modelFile)
    error('Simulink model not found:\n%s', modelFile);
end

open_system(modelName);

%% --------- SIMULATION PARAMETERS ---------
% These are the same structure used before.
% If your model uses different parameters, change here only.

is_real_robot = 0;

Ts = 0.04;
r  = 0.03;
d  = 0.165;

dist_threshold = 0.05;

% Controller gains
% Keep these as your current report/simulation settings.
k_x = 0.5;
k_y = 1.2;
k_theta = 0.3;

% Limits
v_max = 0.22;
omega_max = 1.3;

% Goals
q_goal_task2 = q_goal_task1;   % important: keep target fixed for Task 1 simulation only

% Map coordinates, needed if model requires them
x_coords = [-2.90, -1.60, -1.50, -1.40, -0.60, 0.20, 1.00, 1.45, 1.80, 2.20, 2.90];
y_coords = [-1.50, -1.10, -0.70, -0.30, -0.20, 0.10, 0.20, 1.50];

% Run slightly longer than real Task 1, but not too much
simStopTime = max(t_real(end), 12.5);

fprintf('\nRunning simulation until %.3f s...\n', simStopTime);

set_param(modelName, 'StopTime', num2str(simStopTime));

out = sim(modelName);

%% --------- CHECK SIMULATION OUTPUTS ---------
fprintf('\nSimulation output signals:\n');
disp(out.who);

requiredFields = {'q_est','q_actual','q_target','v','omega','omega_wheels'};

for i = 1:numel(requiredFields)
    if ~isprop(out, requiredFields{i})
        error('Missing simulation output: %s. You need to add/export this signal in Simulink.', requiredFields{i});
    end
end

%% --------- EXPORT SIMULATION DATA ---------
Dsim = struct();

Dsim.q_est        = forceCols(getRaw(out.q_est), 3);
Dsim.q_actual     = forceCols(getRaw(out.q_actual), 3);
Dsim.q_target     = forceCols(getRaw(out.q_target), 3);
Dsim.v            = getRaw(out.v);              Dsim.v = Dsim.v(:);
Dsim.omega        = getRaw(out.omega);          Dsim.omega = Dsim.omega(:);
Dsim.omega_wheels = forceCols(getRaw(out.omega_wheels), 2);

if isprop(out, 'q_est_imu')
    Dsim.q_est_imu = forceCols(getRaw(out.q_est_imu), 3);
else
    Dsim.q_est_imu = NaN(size(Dsim.q_est));
end

if isprop(out, 'u_actual')
    Dsim.u_actual = forceCols(getRaw(out.u_actual), 2);
else
    Dsim.u_actual = NaN(length(Dsim.v), 2);
end

if isprop(out, 'tout')
    Dsim.t = out.tout(:);
elseif isprop(out, 't')
    Dsim.t = getRaw(out.t); Dsim.t = Dsim.t(:);
else
    Dsim.t = (0:size(Dsim.q_actual,1)-1).' * Ts;
end

Nsim = min([length(Dsim.t), size(Dsim.q_actual,1), size(Dsim.q_est,1), ...
            size(Dsim.q_target,1), length(Dsim.v), length(Dsim.omega), ...
            size(Dsim.omega_wheels,1)]);

fieldsSim = fieldnames(Dsim);
for i = 1:numel(fieldsSim)
    f = fieldsSim{i};
    if isnumeric(Dsim.(f)) && size(Dsim.(f),1) >= Nsim
        Dsim.(f) = Dsim.(f)(1:Nsim,:);
    end
end

% Crop simulation after it reaches Task 1 goal, plus small margin
dist_sim = vecnorm(Dsim.q_actual(:,1:2) - q_goal_task1(1:2).', 2, 2);
idxReach = find(dist_sim <= dist_threshold, 1, 'first');

if ~isempty(idxReach)
    idxEndSim = min(length(Dsim.t), idxReach + round(0.8/Ts));
else
    idxEndSim = length(Dsim.t);
end

for i = 1:numel(fieldsSim)
    f = fieldsSim{i};
    if isnumeric(Dsim.(f)) && size(Dsim.(f),1) >= idxEndSim
        Dsim.(f) = Dsim.(f)(1:idxEndSim,:);
    end
end

Dsim.t = Dsim.t - Dsim.t(1);

Dsim_task1_simulation = Dsim;
simSavePath = fullfile(scriptFolder, 'exported_task1_simulation_test3.mat');
save(simSavePath, 'Dsim_task1_simulation');

fprintf('\nSaved simulation data:\n%s\n', simSavePath);

%% ============================================================
% METRICS
% ============================================================

wrap = @(a) atan2(sin(a), cos(a));

% Real final errors using reference
q_ref_end = q_ref_real(end,:);
real_pos_err = norm(q_ref_end(1:2).' - q_goal_task1(1:2));
real_theta_err = abs(wrap(q_goal_task1(3) - q_ref_end(3)));

% q_est vs reference
e_qest_xy = vecnorm(q_est_real(:,1:2) - q_ref_real(:,1:2), 2, 2);
e_qest_theta = abs(wrap(q_ref_real(:,3) - q_est_real(:,3)));

% q_imu vs reference
e_imu_xy = vecnorm(q_imu_real(:,1:2) - q_ref_real(:,1:2), 2, 2);
e_imu_theta = abs(wrap(q_ref_real(:,3) - q_imu_real(:,3)));

% Simulation final errors
q_sim_end = Dsim.q_actual(end,:);
sim_pos_err = norm(q_sim_end(1:2).' - q_goal_task1(1:2));
sim_theta_err = abs(wrap(q_goal_task1(3) - q_sim_end(3)));

Summary = table( ...
    real_pos_err, real_theta_err, rad2deg(real_theta_err), ...
    mean(e_qest_xy,'omitnan'), max(e_qest_xy,[],'omitnan'), ...
    mean(e_imu_xy,'omitnan'), max(e_imu_xy,[],'omitnan'), ...
    sim_pos_err, sim_theta_err, rad2deg(sim_theta_err), ...
    max(abs(omega_wheels_real(:)),[],'omitnan'), ...
    max(abs(Dsim.omega_wheels(:)),[],'omitnan'), ...
    'VariableNames', { ...
    'RealFinalPosErr_m','RealFinalThetaErr_rad','RealFinalThetaErr_deg', ...
    'MeanQestRefXY_m','MaxQestRefXY_m', ...
    'MeanIMURefXY_m','MaxIMURefXY_m', ...
    'SimFinalPosErr_m','SimFinalThetaErr_rad','SimFinalThetaErr_deg', ...
    'MaxRealCommandedWheelSpeed','MaxSimCommandedWheelSpeed'} ...
);

disp(Summary);

summaryPath = fullfile(plotsFolder, 'task1_summary_metrics.csv');
writetable(Summary, summaryPath);

%% ============================================================
% PLOT SETTINGS
% ============================================================

lwReal = 2.2;
lwSim  = 1.8;
lwRef  = 1.6;
lwIMU  = 1.5;

fontSize = 11;
titleSize = 13;

figW = 1100;
figH = 720;

tMax = max(t_real(end), Dsim.t(end));

%% ============================================================
% FIGURE 1 — XY TRAJECTORY
% ============================================================

fig = figure('Color','w','Position',[100 100 figW figH]);

plot(q_ref_real(:,1), q_ref_real(:,2), 'LineWidth', lwReal); hold on;
plot(q_est_real(:,1), q_est_real(:,2), '--', 'LineWidth', lwRef);
plot(Dsim.q_actual(:,1), Dsim.q_actual(:,2), '-.', 'LineWidth', lwSim);
plot(q_goal_task1(1), q_goal_task1(2), 'kx', 'MarkerSize', 11, 'LineWidth', 2.2);
plot(q_ref_real(1,1), q_ref_real(1,2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.8);

grid on; axis equal;
xlabel('x [m]', 'FontSize', fontSize);
ylabel('y [m]', 'FontSize', fontSize);
title('Task 1 - XY Trajectory Comparison', 'FontSize', titleSize, 'FontWeight','bold');

legend(refName, 'EKF/localizer estimate', 'Simulation', 'Goal', 'Start', ...
       'Location','eastoutside');

set(gca, 'FontSize', fontSize);
exportgraphics(fig, fullfile(plotsFolder, 'task1_xy_trajectory.pdf'), 'ContentType','vector');
exportgraphics(fig, fullfile(plotsFolder, 'task1_xy_trajectory.png'), 'Resolution',300);

%% ============================================================
% FIGURE 2 — STATE EVOLUTION
% ============================================================

fig = figure('Color','w','Position',[100 100 figW figH]);

subplot(3,1,1);
plot(t_real, q_ref_real(:,1), 'LineWidth', lwReal); hold on;
plot(t_real, q_est_real(:,1), '--', 'LineWidth', lwRef);
plot(Dsim.t, Dsim.q_actual(:,1), '-.', 'LineWidth', lwSim);
grid on; xlim([0 tMax]);
ylabel('x [m]');
title('Task 1 - State Evolution', 'FontWeight','bold');
legend(refName, 'EKF/localizer estimate', 'Simulation', 'Location','eastoutside');

subplot(3,1,2);
plot(t_real, q_ref_real(:,2), 'LineWidth', lwReal); hold on;
plot(t_real, q_est_real(:,2), '--', 'LineWidth', lwRef);
plot(Dsim.t, Dsim.q_actual(:,2), '-.', 'LineWidth', lwSim);
yline(q_goal_task1(2), ':', 'Goal', 'LabelHorizontalAlignment','right');
grid on; xlim([0 tMax]);
ylabel('y [m]');

subplot(3,1,3);
plot(t_real, q_ref_real(:,3), 'LineWidth', lwReal); hold on;
plot(t_real, q_est_real(:,3), '--', 'LineWidth', lwRef);
plot(Dsim.t, Dsim.q_actual(:,3), '-.', 'LineWidth', lwSim);
yline(q_goal_task1(3), ':', 'Goal', 'LabelHorizontalAlignment','right');
grid on; xlim([0 tMax]);
ylabel('\theta [rad]');
xlabel('Time [s]');

set(findall(fig,'-property','FontSize'), 'FontSize', fontSize);
exportgraphics(fig, fullfile(plotsFolder, 'task1_state_evolution.pdf'), 'ContentType','vector');
exportgraphics(fig, fullfile(plotsFolder, 'task1_state_evolution.png'), 'Resolution',300);

%% ============================================================
% FIGURE 3 — REGULATION ERRORS
% ============================================================

e_pos_real = vecnorm(q_ref_real(:,1:2) - q_goal_task1(1:2).', 2, 2);
e_theta_real = abs(wrap(q_goal_task1(3) - q_ref_real(:,3)));

e_pos_sim = vecnorm(Dsim.q_actual(:,1:2) - q_goal_task1(1:2).', 2, 2);
e_theta_sim = abs(wrap(q_goal_task1(3) - Dsim.q_actual(:,3)));

fig = figure('Color','w','Position',[100 100 figW figH]);

subplot(2,1,1);
plot(t_real, e_pos_real, 'LineWidth', lwReal); hold on;
plot(Dsim.t, e_pos_sim, '-.', 'LineWidth', lwSim);
yline(0.05, ':', '5 cm threshold', 'LabelHorizontalAlignment','right');
grid on; xlim([0 tMax]);
ylabel('Position error [m]');
title('Task 1 - Regulation Errors', 'FontWeight','bold');
legend(refName, 'Simulation', 'Location','eastoutside');

subplot(2,1,2);
plot(t_real, e_theta_real, 'LineWidth', lwReal); hold on;
plot(Dsim.t, e_theta_sim, '-.', 'LineWidth', lwSim);
yline(0.05, ':', '0.05 rad threshold', 'LabelHorizontalAlignment','right');
grid on; xlim([0 tMax]);
ylabel('Orientation error [rad]');
xlabel('Time [s]');

set(findall(fig,'-property','FontSize'), 'FontSize', fontSize);
exportgraphics(fig, fullfile(plotsFolder, 'task1_regulation_errors.pdf'), 'ContentType','vector');
exportgraphics(fig, fullfile(plotsFolder, 'task1_regulation_errors.png'), 'Resolution',300);

%% ============================================================
% FIGURE 4 — POSE ESTIMATION DIAGNOSTIC
% ============================================================

fig = figure('Color','w','Position',[100 100 figW figH]);

subplot(3,1,1);
plot(t_real, q_ref_real(:,1), 'LineWidth', lwReal); hold on;
plot(t_real, q_est_real(:,1), '--', 'LineWidth', lwRef);
plot(t_real, q_imu_real(:,1), ':', 'LineWidth', lwIMU);
grid on; xlim([0 t_real(end)]);
ylabel('x [m]');
title('Task 1 - Pose Estimation Diagnostic', 'FontWeight','bold');
legend(refName, 'EKF/localizer estimate', 'Odometry/IMU estimate', 'Location','eastoutside');

subplot(3,1,2);
plot(t_real, q_ref_real(:,2), 'LineWidth', lwReal); hold on;
plot(t_real, q_est_real(:,2), '--', 'LineWidth', lwRef);
plot(t_real, q_imu_real(:,2), ':', 'LineWidth', lwIMU);
grid on; xlim([0 t_real(end)]);
ylabel('y [m]');

subplot(3,1,3);
plot(t_real, q_ref_real(:,3), 'LineWidth', lwReal); hold on;
plot(t_real, q_est_real(:,3), '--', 'LineWidth', lwRef);
plot(t_real, q_imu_real(:,3), ':', 'LineWidth', lwIMU);
grid on; xlim([0 t_real(end)]);
ylabel('\theta [rad]');
xlabel('Time [s]');

set(findall(fig,'-property','FontSize'), 'FontSize', fontSize);
exportgraphics(fig, fullfile(plotsFolder, 'task1_pose_estimation_diagnostic.pdf'), 'ContentType','vector');
exportgraphics(fig, fullfile(plotsFolder, 'task1_pose_estimation_diagnostic.png'), 'Resolution',300);

%% ============================================================
% FIGURE 5 — CONTROL INPUTS
% ============================================================

fig = figure('Color','w','Position',[100 100 figW figH]);

subplot(2,1,1);
plot(t_real, v_real, 'LineWidth', lwReal); hold on;
plot(Dsim.t, Dsim.v, '-.', 'LineWidth', lwSim);
grid on; xlim([0 tMax]);
ylabel('v [m/s]');
title('Task 1 - Control Inputs', 'FontWeight','bold');
legend('Real command', 'Simulation command', 'Location','eastoutside');

subplot(2,1,2);
plot(t_real, omega_real, 'LineWidth', lwReal); hold on;
plot(Dsim.t, Dsim.omega, '-.', 'LineWidth', lwSim);
grid on; xlim([0 tMax]);
ylabel('\omega [rad/s]');
xlabel('Time [s]');

set(findall(fig,'-property','FontSize'), 'FontSize', fontSize);
exportgraphics(fig, fullfile(plotsFolder, 'task1_control_inputs.pdf'), 'ContentType','vector');
exportgraphics(fig, fullfile(plotsFolder, 'task1_control_inputs.png'), 'Resolution',300);

%% ============================================================
% FIGURE 6 — WHEEL SPEEDS
% ============================================================

fig = figure('Color','w','Position',[100 100 figW figH]);

subplot(2,1,1);
plot(t_real, omega_wheels_real(:,1), 'LineWidth', lwReal); hold on;
plot(t_real, bot_wheelspeed_real(:,1), '--', 'LineWidth', lwRef);
plot(Dsim.t, Dsim.omega_wheels(:,1), '-.', 'LineWidth', lwSim);
yline(10, ':', '+10 rad/s');
yline(-10, ':', '-10 rad/s');
grid on; xlim([0 tMax]);
ylabel('\omega_L [rad/s]');
title('Task 1 - Wheel Speeds', 'FontWeight','bold');
legend('Real command', 'Measured wheel speed', 'Simulation command', 'Location','eastoutside');

subplot(2,1,2);
plot(t_real, omega_wheels_real(:,2), 'LineWidth', lwReal); hold on;
plot(t_real, bot_wheelspeed_real(:,2), '--', 'LineWidth', lwRef);
plot(Dsim.t, Dsim.omega_wheels(:,2), '-.', 'LineWidth', lwSim);
yline(10, ':', '+10 rad/s');
yline(-10, ':', '-10 rad/s');
grid on; xlim([0 tMax]);
ylabel('\omega_R [rad/s]');
xlabel('Time [s]');

set(findall(fig,'-property','FontSize'), 'FontSize', fontSize);
exportgraphics(fig, fullfile(plotsFolder, 'task1_wheel_speeds.pdf'), 'ContentType','vector');
exportgraphics(fig, fullfile(plotsFolder, 'task1_wheel_speeds.png'), 'Resolution',300);

fprintf('\nAll Task 1 plots saved in:\n%s\n', plotsFolder);

%% ============================================================
% Helper functions
% ============================================================

function X = getRaw(sig)

    if isa(sig,'timeseries')
        X = squeeze(sig.Data);

    elseif isnumeric(sig)
        X = squeeze(sig);

    elseif isstruct(sig) && isfield(sig,'signals')
        X = squeeze(sig.signals.values);

    elseif isa(sig,'Simulink.SimulationData.Signal')
        X = squeeze(sig.Values.Data);

    else
        error('Unknown signal format.');
    end
end

function X = forceCols(X, ncols)

    X = squeeze(X);

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Expected %d columns, got size %dx%d.', ncols, size(X,1), size(X,2));
    end
end


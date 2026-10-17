clear; clc; close all;

%% ============================================================
% Final plotting script for report figures
%
% Required files in the same folder:
%   test3_task2_full_outputs.mat
%   exported_task2_simulation.mat
%
% Output folder:
%   ./plots_task2
%
% Generated figures:
%   task2_01_xy_map.pdf / .png
%   task2_02_xy_clean.pdf / .png
%   task2_03_pose_states.pdf / .png
%   task2_04_localization_error.pdf / .png
%   task2_05_control_inputs.pdf / .png
%   task2_06_wheel_speeds.pdf / .png
% ============================================================

%% ---------------- USER SETTINGS ----------------
showFigures = true;

figW = 1100;
figH = 820;
fontSize = 12;

Ts = 0.04;

r = 0.03;
d = 0.165;

omegaLimit = 10;

%% ---------------- WHITE FIGURE STYLE ----------------
set(groot, 'defaultFigureColor', 'w');
set(groot, 'defaultAxesColor', 'w');
set(groot, 'defaultAxesXColor', 'k');
set(groot, 'defaultAxesYColor', 'k');
set(groot, 'defaultTextColor', 'k');
set(groot, 'defaultAxesGridColor', [0.85 0.85 0.85]);
set(groot, 'defaultAxesMinorGridColor', [0.92 0.92 0.92]);
set(groot, 'defaultAxesGridAlpha', 0.35);
set(groot, 'defaultAxesLineWidth', 1.0);
set(groot, 'defaultFigureWindowStyle', 'normal');

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
realFile = fullfile(scriptFolder, 'test3_task2_full_outputs.mat');
simFile  = fullfile(scriptFolder, 'exported_task2_simulation.mat');

plotsFolder = fullfile(scriptFolder, 'plots_task2');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

% Remove old .fig files because only PNG and PDF are needed.
oldFigFiles = dir(fullfile(plotsFolder, '*.fig'));
for k = 1:numel(oldFigFiles)
    delete(fullfile(plotsFolder, oldFigFiles(k).name));
end

if ~isfile(realFile)
    error('Real data file not found:\n%s', realFile);
end

if ~isfile(simFile)
    error('Simulation data file not found:\n%s', simFile);
end

%% ============================================================
% LOAD REAL DATA
% ============================================================

Sreal = load(realFile);

if isfield(Sreal, 'D_task2')
    Dreal = Sreal.D_task2;
else
    error('D_task2 not found inside:\n%s', realFile);
end

q_localizer = ensureNxM(Dreal.q_est, 3);
q_target_real = ensureNxM(Dreal.q_target, 3);

if isfield(Dreal, 'Y_viscon_ground_trith')
    q_vicon = ensureNxM(Dreal.Y_viscon_ground_trith, 3);
elseif isfield(Dreal, 'Y_vicon_ground_truth')
    q_vicon = ensureNxM(Dreal.Y_vicon_ground_truth, 3);
elseif isfield(Dreal, 'Y_viscon')
    q_vicon = ensureNxM(Dreal.Y_viscon, 3);
else
    error('No Vicon signal found in real data.');
end

if isfield(Dreal, 't')
    t_real = Dreal.t(:);
elseif isfield(Dreal, 'tout')
    t_real = Dreal.tout(:);
else
    t_real = (0:size(q_vicon,1)-1).' * Ts;
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
    omega_wheels_real = ensureNxM(Dreal.omega_wheels, 2);
else
    omega_wheels_real = wheelsFromVOmega(v_real, omega_real, r, d);
end

if isfield(Dreal, 'bot_wheelspeed')
    bot_wheelspeed_real = ensureNxM(Dreal.bot_wheelspeed, 2);
else
    bot_wheelspeed_real = NaN(size(omega_wheels_real));
end

%% ---------------- ALIGN REAL LENGTHS ----------------
Nreal = min([
    length(t_real)
    size(q_vicon,1)
    size(q_localizer,1)
    size(q_target_real,1)
    length(v_real)
    length(omega_real)
    size(omega_wheels_real,1)
    size(bot_wheelspeed_real,1)
]);

t_real = t_real(1:Nreal);
q_vicon = q_vicon(1:Nreal,:);
q_localizer = q_localizer(1:Nreal,:);
q_target_real = q_target_real(1:Nreal,:);
v_real = v_real(1:Nreal);
omega_real = omega_real(1:Nreal);
omega_wheels_real = omega_wheels_real(1:Nreal,:);
bot_wheelspeed_real = bot_wheelspeed_real(1:Nreal,:);

%% ============================================================
% LOAD AND ROBUSTLY TIME-ALIGN SIMULATION DATA
% ============================================================

Ssim = load(simFile);

if isfield(Ssim, 'Dsim_task2')
    Dsim = Ssim.Dsim_task2;
else
    error('Dsim_task2 not found inside:\n%s', simFile);
end

% ---------------- Raw simulation signals ----------------
q_sim_raw = ensureNxM(Dsim.q_actual, 3);

if isfield(Dsim, 'q_est')
    q_sim_localizer_raw = ensureNxM(Dsim.q_est, 3);
else
    q_sim_localizer_raw = q_sim_raw;
end

if isfield(Dsim, 'q_target')
    q_target_sim_raw = ensureNxM(Dsim.q_target, 3);
else
    q_target_sim_raw = q_target_real;
end

if isfield(Dsim, 't')
    t_sim_raw = Dsim.t(:);
elseif isfield(Dsim, 'tout')
    t_sim_raw = Dsim.tout(:);
else
    t_sim_raw = (0:size(q_sim_raw,1)-1).' * Ts;
end

t_sim_raw = t_sim_raw - t_sim_raw(1);

if isfield(Dsim, 'v')
    v_sim_raw = Dsim.v(:);
else
    v_sim_raw = NaN(size(t_sim_raw));
end

if isfield(Dsim, 'omega')
    omega_sim_raw = Dsim.omega(:);
else
    omega_sim_raw = NaN(size(t_sim_raw));
end

if isfield(Dsim, 'omega_wheels')
    omega_wheels_sim_raw = ensureNxM(Dsim.omega_wheels, 2);
else
    omega_wheels_sim_raw = wheelsFromVOmega(v_sim_raw, omega_sim_raw, r, d);
end

% ---------------- First safety alignment by sample length ----------------
Nsim_raw = min([
    length(t_sim_raw)
    size(q_sim_raw,1)
    size(q_sim_localizer_raw,1)
    size(q_target_sim_raw,1)
    length(v_sim_raw)
    length(omega_sim_raw)
    size(omega_wheels_sim_raw,1)
]);

t_sim_raw = t_sim_raw(1:Nsim_raw);
q_sim_raw = q_sim_raw(1:Nsim_raw,:);
q_sim_localizer_raw = q_sim_localizer_raw(1:Nsim_raw,:);
q_target_sim_raw = q_target_sim_raw(1:Nsim_raw,:);
v_sim_raw = v_sim_raw(1:Nsim_raw);
omega_sim_raw = omega_sim_raw(1:Nsim_raw);
omega_wheels_sim_raw = omega_wheels_sim_raw(1:Nsim_raw,:);

% ---------------- Angle wrapping before interpolation ----------------
q_sim_raw(:,3) = wrapToPiLocal(q_sim_raw(:,3));
q_sim_localizer_raw(:,3) = wrapToPiLocal(q_sim_localizer_raw(:,3));
q_target_sim_raw(:,3) = wrapToPiLocal(q_target_sim_raw(:,3));

% ---------------- Robust time normalization ----------------
% The simulation may have a slightly different duration/sample count.
% For comparison plots, map the simulation time interval onto the real
% experiment time interval, then interpolate all simulation signals on t_real.

t_sim_scaled = robustNormalizeTime(t_sim_raw, t_real(end));

% Use the real timeline as the master comparison timeline.
t_sim = t_real;
Nsim = length(t_sim);

q_sim = interpPoseOnTime(t_sim_scaled, q_sim_raw, t_sim);
q_sim_localizer = interpPoseOnTime(t_sim_scaled, q_sim_localizer_raw, t_sim);
q_target_sim = interpPoseOnTime(t_sim_scaled, q_target_sim_raw, t_sim);

v_sim = interpSignalOnTime(t_sim_scaled, v_sim_raw, t_sim);
omega_sim = interpSignalOnTime(t_sim_scaled, omega_sim_raw, t_sim);
omega_wheels_sim = interpSignalOnTime(t_sim_scaled, omega_wheels_sim_raw, t_sim);

% ---------------- Light robustness for plotting only ----------------
% This removes tiny interpolation artifacts but does not change real data.
q_sim(:,1) = movmedian(q_sim(:,1), 3, 'omitnan');
q_sim(:,2) = movmedian(q_sim(:,2), 3, 'omitnan');
q_sim(:,3) = wrapToPiLocal(movmedian(q_sim(:,3), 3, 'omitnan'));

q_sim_localizer(:,1) = movmedian(q_sim_localizer(:,1), 3, 'omitnan');
q_sim_localizer(:,2) = movmedian(q_sim_localizer(:,2), 3, 'omitnan');
q_sim_localizer(:,3) = wrapToPiLocal(movmedian(q_sim_localizer(:,3), 3, 'omitnan'));

fprintf('\nSimulation timeline alignment:\n');
fprintf('Raw simulation duration:     %.3f s\n', t_sim_raw(end));
fprintf('Real experiment duration:    %.3f s\n', t_real(end));
fprintf('Aligned simulation duration: %.3f s\n', t_sim(end));
fprintf('Simulation samples after alignment: %d\n', Nsim);
fprintf('NOTE: Simulation signals are resampled on the real experimental time vector for plotting.\n');
fprintf('This affects visualization timing, not the original simulated trajectory shape.\n');

%% ---------------- ALIGN SIMULATION LENGTHS ----------------
Nsim = min([
    length(t_sim)
    size(q_sim,1)
    size(q_sim_localizer,1)
    size(q_target_sim,1)
    length(v_sim)
    length(omega_sim)
    size(omega_wheels_sim,1)
]);

t_sim = t_sim(1:Nsim);
q_sim = q_sim(1:Nsim,:);
q_sim_localizer = q_sim_localizer(1:Nsim,:);
q_target_sim = q_target_sim(1:Nsim,:);
v_sim = v_sim(1:Nsim);
omega_sim = omega_sim(1:Nsim);
omega_wheels_sim = omega_wheels_sim(1:Nsim,:);

%% ============================================================
% WAYPOINTS AND MAP
% ============================================================

q_wp1   = [-0.20; -1.10; 0];
q_wp2   = [-0.20; -0.30; pi/2];
q_goal2 = [ 1.00; -0.30; 0];

redArea = [-0.60, -1.50, 0.80, 1.60];

orangeArea = [1.00, -1.50, 1.90, 3.00];
initArea   = [-2.90, 0.10, 1.40, 1.40];

obstacles = [
    -1.50, -0.70, 0.90, 0.80
     0.20, -1.50, 0.80, 0.80
     1.80, -0.20, 0.40, 0.40
];

%% ============================================================
% ANGLE WRAPPING
% ============================================================

q_vicon(:,3)        = wrapToPiLocal(q_vicon(:,3));
q_localizer(:,3)    = wrapToPiLocal(q_localizer(:,3));
q_target_real(:,3)  = wrapToPiLocal(q_target_real(:,3));

q_sim(:,3)           = wrapToPiLocal(q_sim(:,3));
q_sim_localizer(:,3) = wrapToPiLocal(q_sim_localizer(:,3));
q_target_sim(:,3)    = wrapToPiLocal(q_target_sim(:,3));

%% ============================================================
% RED AREA INTERVALS
% ============================================================

in_red_real = isInsideRect(q_vicon(:,1:2), redArea);
in_red_sim  = isInsideRect(q_sim(:,1:2), redArea);

realRedIntervals = maskIntervals(t_real, in_red_real);
simRedIntervals  = maskIntervals(t_sim, in_red_sim);

%% ============================================================
% FIX MEASURED WHEEL-SPEED COLUMN ORDER
% ============================================================

err_same = mean(vecnorm(bot_wheelspeed_real - omega_wheels_real, 2, 2), 'omitnan');
err_swap = mean(vecnorm([bot_wheelspeed_real(:,2), bot_wheelspeed_real(:,1)] - omega_wheels_real, 2, 2), 'omitnan');

fprintf('\nWheel speed column check:\n');
fprintf('Mean error without swap = %.4f rad/s\n', err_same);
fprintf('Mean error with swap    = %.4f rad/s\n', err_swap);

if err_swap < err_same
    fprintf('Result: measured wheel-speed columns are swapped. Using swapped order.\n');
    bot_wheelspeed_LR = [bot_wheelspeed_real(:,2), bot_wheelspeed_real(:,1)];
else
    fprintf('Result: measured wheel-speed columns are not swapped. Using original order.\n');
    bot_wheelspeed_LR = bot_wheelspeed_real;
end

%% ============================================================
% LIGHT FILTERING FOR PLOTTING ONLY
% Raw data are kept for metrics.
% Filtered variables are used only to make report figures cleaner.
% ============================================================

filterWindow = 5;

v_real_plot = cleanForPlot(v_real, filterWindow, [-0.25, 0.25]);
omega_real_plot = cleanForPlot(omega_real, filterWindow, [-2.5, 2.5]);

omega_wheels_real_plot = cleanForPlot(omega_wheels_real, filterWindow, [-10.2, 10.2]);
bot_wheelspeed_LR_plot = cleanForPlot(bot_wheelspeed_LR, filterWindow, [-10.8, 10.8]);

v_sim_plot = cleanForPlot(v_sim, filterWindow, [-0.25, 0.25]);
omega_sim_plot = cleanForPlot(omega_sim, filterWindow, [-2.5, 2.5]);
omega_wheels_sim_plot = cleanForPlot(omega_wheels_sim, filterWindow, [-10.8, 10.8]);

%% ============================================================
% ERRORS AND SUMMARY METRICS
% ============================================================

wrap = @(a) atan2(sin(a), cos(a));

e_real_goal_pos = vecnorm(q_vicon(:,1:2) - q_goal2(1:2).', 2, 2);
e_real_goal_theta = abs(wrap(q_goal2(3) - q_vicon(:,3)));

e_sim_goal_pos = vecnorm(q_sim(:,1:2) - q_goal2(1:2).', 2, 2);
e_sim_goal_theta = abs(wrap(q_goal2(3) - q_sim(:,3)));

e_localizer_vicon_xy = vecnorm(q_localizer(:,1:2) - q_vicon(:,1:2), 2, 2);
e_localizer_vicon_theta = wrap(q_localizer(:,3) - q_vicon(:,3));

e_sim_localizer_actual_xy = vecnorm(q_sim_localizer(:,1:2) - q_sim(:,1:2), 2, 2);
e_sim_localizer_actual_theta = wrap(q_sim_localizer(:,3) - q_sim(:,3));

Summary = table( ...
    t_real(end), t_sim(end), ...
    e_real_goal_pos(end), e_real_goal_theta(end), rad2deg(e_real_goal_theta(end)), ...
    e_sim_goal_pos(end), e_sim_goal_theta(end), rad2deg(e_sim_goal_theta(end)), ...
    sqrt(mean(e_localizer_vicon_xy.^2,'omitnan')), max(e_localizer_vicon_xy,[],'omitnan'), ...
    sqrt(mean(e_sim_localizer_actual_xy.^2,'omitnan')), max(e_sim_localizer_actual_xy,[],'omitnan'), ...
    max(abs(omega_wheels_real(:)),[],'omitnan'), ...
    max(abs(bot_wheelspeed_LR(:)),[],'omitnan'), ...
    max(abs(omega_wheels_sim(:)),[],'omitnan'), ...
    'VariableNames', { ...
    'RealDuration_s','AlignedSimulationDuration_s', ...
    'RealFinalPosErr_m','RealFinalThetaErr_rad','RealFinalThetaErr_deg', ...
    'SimulationFinalPosErr_m','SimulationFinalThetaErr_rad','SimulationFinalThetaErr_deg', ...
    'RealLocalizerRMSE_m','RealLocalizerMaxErr_m', ...
    'SimulationLocalizerRMSE_m','SimulationLocalizerMaxErr_m', ...
    'MaxRealWheelCommand_rad_s', ...
    'MaxMeasuredWheelSpeed_rad_s', ...
    'MaxSimulationWheelCommand_rad_s'} ...
);

fprintf('\nSummary metrics:\n');
disp(Summary);

writetable(Summary, fullfile(plotsFolder, 'task2_summary_metrics.csv'));

fprintf('\nReal red-area intervals [s]:\n');
disp(realRedIntervals);

fprintf('Simulation red-area intervals [s]:\n');
disp(simRedIntervals);

%% ============================================================
% PLOT STYLE
% ============================================================

cVicon = [0.0000 0.4470 0.7410];
cLoc   = [0.8500 0.3250 0.0980];
cSim   = [0.0000 0.5000 0.0000];
cMeas  = [0.4940 0.1840 0.5560];
cThr   = [0.1500 0.1500 0.1500];
cRed   = [1.0000 0.0000 0.0000];

lwVicon = 2.3;
lwLoc   = 1.9;
lwSim   = 2.0;
lwMeas  = 1.5;
lwRef   = 1.4;
lwThr   = 1.1;

% Since the simulated signals are resampled on the real timeline,
% the report time axis is the experimental time axis.
xMaxPlot = t_real(end);

%% ============================================================
% FIGURE 1 — XY TRAJECTORY WITH MAP
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
hold on; grid on; axis equal;

rectangle('Position',[-2.90, -1.50, 5.80, 3.00], ...
          'EdgeColor','k', 'LineWidth', 1.2, ...
          'HandleVisibility','off');

addRect(initArea,   [0.60 0.75 1.00], 0.12);
addRect(redArea,    [1.00 0.00 0.00], 0.08);
addRect(orangeArea, [1.00 0.50 0.00], 0.10);

for i = 1:size(obstacles,1)
    rectangle('Position', obstacles(i,:), ...
              'FaceColor', [0.55 0.55 0.55], ...
              'EdgeColor', 'k', ...
              'LineWidth', 1.0, ...
              'HandleVisibility','off');
end

pRef = plot(q_target_real(:,1), q_target_real(:,2), 'k--', ...
    'LineWidth', lwRef);

pVicon = plot(q_vicon(:,1), q_vicon(:,2), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon);

pLoc = plot(q_localizer(:,1), q_localizer(:,2), '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

pSim = plot(q_sim(:,1), q_sim(:,2), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

pStart = plot(q_vicon(1,1), q_vicon(1,2), 'ko', ...
    'MarkerFaceColor', [0 0.9 0], ...
    'MarkerSize', 8, ...
    'LineWidth', 1.8);

pWp1 = plot(q_wp1(1), q_wp1(2), 'ko', ...
    'MarkerFaceColor','w', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.6);

pWp2 = plot(q_wp2(1), q_wp2(2), 'ks', ...
    'MarkerFaceColor','w', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.6);

pGoal = plot(q_goal2(1), q_goal2(2), 'kx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2.3);

xlabel('x [m]');
ylabel('y [m]');
title('XY trajectory with map', ...
      'FontWeight','bold', ...
      'Units','normalized', ...
      'Position',[0.5 1.055 0]);

legend([pRef pVicon pLoc pSim pStart pWp1 pWp2 pGoal], ...
       {'Desired waypoint path', ...
        'Vicon', ...
        'Localizer estimate', ...
        'Simulation', ...
        'Start', ...
        'Waypoint 1', ...
        'Waypoint 2', ...
        'Goal'}, ...
       'Location','eastoutside');

xlim([-1.90 1.25]);
ylim([-1.35 0.15]);

saveReportFigure(fig, plotsFolder, 'task2_01_xy_map', fontSize, showFigures);

%% ============================================================
% FIGURE 2 — CLEAN XY TRAJECTORY
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
hold on; grid on; axis equal;

pRef = plot(q_target_real(:,1), q_target_real(:,2), 'k--', ...
    'LineWidth', lwRef);

pVicon = plot(q_vicon(:,1), q_vicon(:,2), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon);

pLoc = plot(q_localizer(:,1), q_localizer(:,2), '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

pSim = plot(q_sim(:,1), q_sim(:,2), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

pStart = plot(q_vicon(1,1), q_vicon(1,2), 'ko', ...
    'MarkerFaceColor', [0 0.9 0], ...
    'MarkerSize', 8, ...
    'LineWidth', 1.8);

pWp1 = plot(q_wp1(1), q_wp1(2), 'ko', ...
    'MarkerFaceColor','w', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.6);

pWp2 = plot(q_wp2(1), q_wp2(2), 'ks', ...
    'MarkerFaceColor','w', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.6);

pGoal = plot(q_goal2(1), q_goal2(2), 'kx', ...
    'MarkerSize', 12, ...
    'LineWidth', 2.3);

xlabel('x [m]');
ylabel('y [m]');
title('XY trajectory comparison', ...
      'FontWeight','bold', ...
      'Units','normalized', ...
      'Position',[0.5 1.055 0]);

legend([pRef pVicon pLoc pSim pStart pWp1 pWp2 pGoal], ...
       {'Desired waypoint path', ...
        'Vicon', ...
        'Localizer estimate', ...
        'Simulation', ...
        'Start', ...
        'Waypoint 1', ...
        'Waypoint 2', ...
        'Goal'}, ...
       'Location','eastoutside');

xlim([-1.75 1.15]);
ylim([-1.25 -0.15]);

saveReportFigure(fig, plotsFolder, 'task2_02_xy_clean', fontSize, showFigures);

%% ============================================================
% FIGURE 3 — POSE STATES
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pRef = plot(t_real, q_target_real(:,1), 'k--', 'LineWidth', lwRef); hold on;
pVicon = plot(t_real, q_vicon(:,1), '-', 'Color', cVicon, 'LineWidth', lwVicon);
pLoc = plot(t_real, q_localizer(:,1), '--', 'Color', cLoc, 'LineWidth', lwLoc);
pSim = plot(t_sim, q_sim(:,1), '-.', 'Color', cSim, 'LineWidth', lwSim);
shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('x [m]');
title('Pose states', 'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
plot(t_real, q_target_real(:,2), 'k--', 'LineWidth', lwRef); hold on;
plot(t_real, q_vicon(:,2), '-', 'Color', cVicon, 'LineWidth', lwVicon);
plot(t_real, q_localizer(:,2), '--', 'Color', cLoc, 'LineWidth', lwLoc);
plot(t_sim, q_sim(:,2), '-.', 'Color', cSim, 'LineWidth', lwSim);
shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('y [m]');

nexttile;
plot(t_real, q_target_real(:,3), 'k--', 'LineWidth', lwRef); hold on;
plot(t_real, q_vicon(:,3), '-', 'Color', cVicon, 'LineWidth', lwVicon);
plot(t_real, q_localizer(:,3), '--', 'Color', cLoc, 'LineWidth', lwLoc);
plot(t_sim, q_sim(:,3), '-.', 'Color', cSim, 'LineWidth', lwSim);
shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('\theta [rad]');
xlabel('Time [s]');

lgd = legend(ax1, [pRef pVicon pLoc pSim], ...
    {'Desired waypoint path', 'Vicon', 'Localizer estimate', 'Simulation'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task2_03_pose_states', fontSize, showFigures);

%% ============================================================
% FIGURE 4 — LOCALIZATION ERROR
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pReal = plot(t_real, e_localizer_vicon_xy, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pSim = plot(t_sim, e_sim_localizer_actual_xy, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('Position error [m]');
title('Localization error', 'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
plot(t_real, e_localizer_vicon_theta, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_sim, e_sim_localizer_actual_theta, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('\theta error [rad]');
xlabel('Time [s]');

lgd = legend(ax1, [pReal pSim], ...
    {'Real localization error', 'Simulation localization error'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task2_04_localization_error', fontSize, showFigures);

%% ============================================================
% FIGURE 5 — CONTROL INPUTS
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pReal = plot(t_real, v_real_plot, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pSim = plot(t_sim, v_sim_plot, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('v [m/s]');
title('Control inputs', 'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
plot(t_real, omega_real_plot, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_sim, omega_sim_plot, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('\omega [rad/s]');
xlabel('Time [s]');

lgd = legend(ax1, [pReal pSim], ...
    {'Real command', 'Simulation command'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task2_05_control_inputs', fontSize, showFigures);

%% ============================================================
% FIGURE 6 — WHEEL SPEEDS
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pCmd = plot(t_real, omega_wheels_real_plot(:,1), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pMeas = plot(t_real, bot_wheelspeed_LR_plot(:,1), '--', ...
    'Color', cMeas, 'LineWidth', lwMeas);

pSim = plot(t_sim, omega_wheels_sim_plot(:,1), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline( omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');
yline(-omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');

shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('\omega_L [rad/s]');
title('Wheel speeds', 'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
plot(t_real, omega_wheels_real_plot(:,2), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_real, bot_wheelspeed_LR_plot(:,2), '--', ...
    'Color', cMeas, 'LineWidth', lwMeas);

plot(t_sim, omega_wheels_sim_plot(:,2), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline( omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');
yline(-omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');

shadeIntervals(gca, realRedIntervals, cRed, 0.06);
grid on; xlim([0 xMaxPlot]);
ylabel('\omega_R [rad/s]');
xlabel('Time [s]');

lgd = legend(ax1, [pCmd pMeas pSim], ...
    {'Real command', 'Measured wheel speed', 'Simulation command'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task2_06_wheel_speeds', fontSize, showFigures);

fprintf('\nAll plots saved in:\n%s\n', plotsFolder);
fprintf('\nRecommended LaTeX figures:\n');
fprintf('  task2_01_xy_map.pdf\n');
fprintf('  task2_03_pose_states.pdf\n');
fprintf('  task2_04_localization_error.pdf\n');
fprintf('  task2_05_control_inputs.pdf\n');
fprintf('  task2_06_wheel_speeds.pdf\n');

%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function X = ensureNxM(X, M)

    X = squeeze(X);

    if isvector(X)
        X = X(:).';
    end

    if size(X,2) ~= M && size(X,1) == M
        X = X.';
    end

    if size(X,2) ~= M
        error('Signal does not have expected number of columns. Expected %d columns, got size %dx%d.', ...
            M, size(X,1), size(X,2));
    end
end

function a = wrapToPiLocal(a)

    a = atan2(sin(a), cos(a));
end

function tf = isInsideRect(P, rect)

    x = P(:,1);
    y = P(:,2);

    tf = x >= rect(1) & x <= rect(1)+rect(3) & ...
         y >= rect(2) & y <= rect(2)+rect(4);
end

function intervals = maskIntervals(t, mask)

    mask = mask(:);

    if ~any(mask)
        intervals = zeros(0,2);
        return;
    end

    d = diff([false; mask; false]);
    idxStart = find(d == 1);
    idxEnd   = find(d == -1) - 1;

    intervals = [t(idxStart), t(idxEnd)];
end

function h = addRect(pos, color, alphaVal)

    x = pos(1);
    y = pos(2);
    w = pos(3);
    hgt = pos(4);

    h = patch([x x+w x+w x], ...
              [y y y+hgt y+hgt], ...
              color, ...
              'FaceAlpha', alphaVal, ...
              'EdgeColor','none', ...
              'HandleVisibility','off');
end

function shadeIntervals(ax, intervals, color, alphaVal)

    if isempty(intervals)
        return;
    end

    axes(ax);
    yl = ylim(ax);

    for i = 1:size(intervals,1)

        x1 = intervals(i,1);
        x2 = intervals(i,2);

        patch(ax, ...
            [x1 x2 x2 x1], ...
            [yl(1) yl(1) yl(2) yl(2)], ...
            color, ...
            'FaceAlpha', alphaVal, ...
            'EdgeColor','none', ...
            'HandleVisibility','off');
    end

    patches = findobj(ax,'Type','patch');

    if ~isempty(patches)
        uistack(patches, 'bottom');
    end
end

function fig = newReportFigure(showFigures, figW, figH)

    if showFigures
        fig = figure('Visible','on');
    else
        fig = figure('Visible','off');
    end

    set(fig, ...
        'Color','w', ...
        'Units','pixels', ...
        'Position',[120 80 figW figH], ...
        'WindowStyle','normal', ...
        'InvertHardcopy','off');

    drawnow;
end

function saveReportFigure(fig, plotsFolder, baseName, fontSize, showFigures)

    set(findall(fig,'-property','FontSize'), 'FontSize', fontSize);
    makeFigureWhite(fig);
    removeAxesToolbar(fig);
    fixFigureMargins(fig);

    drawnow;

    exportgraphics(fig, fullfile(plotsFolder, [baseName '.pdf']), ...
        'ContentType','vector', ...
        'BackgroundColor','white');

    exportgraphics(fig, fullfile(plotsFolder, [baseName '.png']), ...
        'Resolution',300, ...
        'BackgroundColor','white');

    fprintf('Saved PDF: %s\n', fullfile(plotsFolder, [baseName '.pdf']));
    fprintf('Saved PNG: %s\n', fullfile(plotsFolder, [baseName '.png']));

    if ~showFigures
        close(fig);
    end
end

function makeFigureWhite(fig)

    set(fig, 'Color', 'w');
    set(fig, 'InvertHardcopy', 'off');

    ax = findall(fig, 'Type', 'axes');

    for k = 1:numel(ax)
        set(ax(k), ...
            'Color', 'w', ...
            'XColor', 'k', ...
            'YColor', 'k', ...
            'GridColor', [0.85 0.85 0.85], ...
            'GridAlpha', 0.35, ...
            'Box', 'on', ...
            'LineWidth', 1.0);
    end

    lgd = findall(fig, 'Type', 'legend');

    for k = 1:numel(lgd)
        set(lgd(k), ...
            'Color', 'w', ...
            'TextColor', 'k', ...
            'EdgeColor', [0.30 0.30 0.30]);
    end
end

function removeAxesToolbar(fig)

    ax = findall(fig, 'Type', 'axes');

    for k = 1:numel(ax)
        try
            disableDefaultInteractivity(ax(k));
        catch
        end

        try
            ax(k).Toolbar.Visible = 'off';
        catch
        end
    end
end

function fixFigureMargins(fig)

    ax = findall(fig, 'Type', 'axes');

    for k = 1:numel(ax)
        try
            ax(k).Title.Units = 'normalized';
            ax(k).Title.Position(2) = max(ax(k).Title.Position(2), 1.04);
        catch
        end
    end
end

function Y = cleanForPlot(X, windowLength, hardLimits)

    Y = double(X);

    if isvector(Y)
        Y = Y(:);
    end

    for j = 1:size(Y,2)

        y = Y(:,j);

        if nargin >= 3 && ~isempty(hardLimits)
            y(y < hardLimits(1) | y > hardLimits(2)) = NaN;
        end

        y = fillmissing(y, 'linear', 'EndValues','nearest');
        y = movmedian(y, windowLength, 'omitnan');

        Y(:,j) = y;
    end
end

function omega_wheels = wheelsFromVOmega(v, omega, r, d)

    v = v(:);
    omega = omega(:);

    omega_L = (v - omega * d/2) / r;
    omega_R = (v + omega * d/2) / r;

    omega_wheels = [omega_L, omega_R];
end

function t_scaled = robustNormalizeTime(t_raw, targetDuration)

    t_raw = t_raw(:);
    t_raw = t_raw - t_raw(1);

    % Remove repeated or invalid time samples
    valid = isfinite(t_raw);
    t_raw = t_raw(valid);

    if numel(t_raw) < 2 || t_raw(end) <= 0
        error('Invalid simulation time vector.');
    end

    t_scaled = t_raw / t_raw(end) * targetDuration;
end

function Xq = interpSignalOnTime(t_src, X_src, t_dst)

    t_src = t_src(:);
    t_dst = t_dst(:);

    X_src = squeeze(X_src);

    if isvector(X_src)
        X_src = X_src(:);
    end

    % Make source time strictly increasing
    [t_src_unique, idxUnique] = unique(t_src, 'stable');
    X_src = X_src(idxUnique,:);

    % Fill missing values before interpolation
    for j = 1:size(X_src,2)
        X_src(:,j) = fillmissing(X_src(:,j), 'linear', 'EndValues','nearest');
    end

    Xq = zeros(length(t_dst), size(X_src,2));

    for j = 1:size(X_src,2)
        Xq(:,j) = interp1(t_src_unique, X_src(:,j), t_dst, ...
                          'linear', 'extrap');
    end
end

function Qq = interpPoseOnTime(t_src, Q_src, t_dst)

    t_src = t_src(:);
    t_dst = t_dst(:);

    Q_src = squeeze(Q_src);

    if size(Q_src,2) ~= 3 && size(Q_src,1) == 3
        Q_src = Q_src.';
    end

    if size(Q_src,2) ~= 3
        error('Pose signal must have 3 columns.');
    end

    % Make source time strictly increasing
    [t_src_unique, idxUnique] = unique(t_src, 'stable');
    Q_src = Q_src(idxUnique,:);

    % Interpolate x and y normally
    Qq = zeros(length(t_dst), 3);

    Qq(:,1) = interp1(t_src_unique, Q_src(:,1), t_dst, 'linear', 'extrap');
    Qq(:,2) = interp1(t_src_unique, Q_src(:,2), t_dst, 'linear', 'extrap');

    % Interpolate theta using unwrap first, then wrap again
    theta_unwrapped = unwrap(Q_src(:,3));
    theta_interp = interp1(t_src_unique, theta_unwrapped, t_dst, ...
                           'linear', 'extrap');

    Qq(:,3) = wrapToPiLocal(theta_interp);
end
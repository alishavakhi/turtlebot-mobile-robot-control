%% plot_task3_simulation_report.m
% Task 3 report plots
%
% This version compares:
%   1) extracted real robot segment:
%        real_task3_from_full.mat
%   2) simulation starting from Task 2 final pose:
%        exported_task3_simulation_from_task2.mat
%
% Important:
% No artificial time shift is used here.
% Both real and simulation are plotted from the beginning of the Task 3 segment.
%
% Time-axis note:
% The exported simulation may continue a few seconds after stop.
% For report-quality figures, all time-domain plots are limited to slightly
% after the last stop marker.

clear; clc; close all;

%% ============================================================
% USER SETTINGS
% ============================================================
realFile = 'real_task3_from_full.mat';
simFile  = 'exported_task3_simulation_from_task2.mat';

plotsFolder = 'plots_task3_real_vs_sim';

showFigures = true;
figW = 1150;
figH = 820;
fontSize = 12;

omegaLimit_default = 10;
filterWindow = 5;

task3_start_xy = [1.45, -0.30];

%% ============================================================
% STYLE
% ============================================================
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

cVicon  = [0.0000 0.4470 0.7410];   % real Vicon
cLoc    = [0.8500 0.3250 0.0980];   % localizer / EKF
cSim    = [0.0000 0.5000 0.0000];   % simulation
cMeas   = [0.4940 0.1840 0.5560];   % measured / loss
cRef    = [0.0000 0.0000 0.0000];   % reference
cThr    = [0.1500 0.1500 0.1500];
cRed    = [1.0000 0.0000 0.0000];
cOrange = [1.0000 0.5000 0.0000];

lwVicon = 2.3;
lwLoc   = 1.9;
lwSim   = 2.0;
lwMeas  = 1.5;
lwRef   = 1.4;
lwThr   = 1.1;

%% ============================================================
% FOLDER SETUP
% ============================================================
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    scriptFolder = pwd;
else
    scriptFolder = fileparts(scriptPath);
end

cd(scriptFolder);

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

if ~isfile(realFile)
    error('Real file not found: %s. Run extract_task3_real_from_full.m first.', realFile);
end

if ~isfile(simFile)
    error('Simulation file not found: %s. Run run_export_task3_simulation.m first.', simFile);
end

%% ============================================================
% LOAD REAL DATA
% ============================================================
Lreal = load(realFile);

if ~isfield(Lreal, 'Dreal_task3')
    error('Dreal_task3 not found inside %s.', realFile);
end

Dreal = Lreal.Dreal_task3;

t_real = Dreal.t(:);
t_real = t_real - t_real(1);

q_real_gt = ensureNxM(Dreal.Y_vicon_ground_truth, 3);
q_real_est = ensureNxM(Dreal.q_est, 3);
q_real_ref = ensureNxM(Dreal.q_target, 3);

if isfield(Dreal, 'Y_vicon_turtlebot')
    q_real_vicon = ensureNxM(Dreal.Y_vicon_turtlebot, 3);
elseif isfield(Dreal, 'Y_viscon')
    q_real_vicon = ensureNxM(Dreal.Y_viscon, 3);
else
    q_real_vicon = q_real_gt;
end

if isfield(Dreal, 'q_est_imu')
    q_real_imu = ensureNxM(Dreal.q_est_imu, 3);
else
    q_real_imu = NaN(size(q_real_est));
end

v_real = Dreal.v(:);
omega_real = Dreal.omega(:);
wheel_real_cmd = ensureNxM(Dreal.omega_wheels, 2);

if isfield(Dreal, 'bot_wheelspeed')
    wheel_real_meas = ensureNxM(Dreal.bot_wheelspeed, 2);
else
    wheel_real_meas = NaN(size(wheel_real_cmd));
end

if isfield(Dreal, 'loss_flag')
    loss_real = Dreal.loss_flag(:) > 0.5;
else
    loss_real = all(abs(q_real_vicon) < 1e-12, 2);
end

if isfield(Dreal, 'vicon_valid')
    valid_real = Dreal.vicon_valid(:) > 0.5;
else
    valid_real = ~loss_real;
end

if isfield(Dreal, 'in_red')
    red_real = Dreal.in_red(:) > 0.5;
else
    red_real = q_real_gt(:,1) >= -0.60 & q_real_gt(:,1) <= 0.20 & ...
               q_real_gt(:,2) >= -1.50 & q_real_gt(:,2) <= 0.10;
end

if isfield(Dreal, 'in_orange')
    orange_real = Dreal.in_orange(:) > 0.5;
else
    orange_real = q_real_gt(:,1) >= 1.00 & q_real_gt(:,1) <= 2.90 & ...
                  q_real_gt(:,2) >= -1.50 & q_real_gt(:,2) <= 1.50;
end

omegaLimit = getFieldDefault(Dreal, 'omega_M', omegaLimit_default);

%% ============================================================
% LOAD SIMULATION DATA
% ============================================================
Lsim = load(simFile);

if ~isfield(Lsim, 'Dsim_task3')
    error('Dsim_task3 not found inside %s.', simFile);
end

Dsim = Lsim.Dsim_task3;

t_sim = Dsim.t(:);
t_sim = t_sim - t_sim(1);

q_sim = ensureNxM(Dsim.q_actual, 3);
q_sim_est = ensureNxM(Dsim.q_est, 3);
q_sim_ref = ensureNxM(Dsim.q_target, 3);

if isfield(Dsim, 'Y_vicon_ground_truth')
    q_sim_gt = ensureNxM(Dsim.Y_vicon_ground_truth, 3);
elseif isfield(Dsim, 'Y_vicon_ground_truth_sim')
    q_sim_gt = ensureNxM(Dsim.Y_vicon_ground_truth_sim, 3);
else
    q_sim_gt = q_sim;
end

v_sim = Dsim.v(:);
omega_sim = Dsim.omega(:);
wheel_sim = ensureNxM(Dsim.omega_wheels, 2);

if isfield(Dsim, 'loss_flag')
    loss_sim = Dsim.loss_flag(:) > 0.5;
else
    loss_sim = false(size(t_sim));
end

if isfield(Dsim, 'in_red')
    red_sim = Dsim.in_red(:) > 0.5;
else
    red_sim = false(size(t_sim));
end

if isfield(Dsim, 'in_orange')
    orange_sim = Dsim.in_orange(:) > 0.5;
else
    orange_sim = true(size(t_sim));
end

%% ============================================================
% ALIGN LENGTHS
% ============================================================
Nreal = min([ ...
    numel(t_real), ...
    size(q_real_gt,1), ...
    size(q_real_est,1), ...
    size(q_real_ref,1), ...
    size(q_real_vicon,1), ...
    size(q_real_imu,1), ...
    numel(v_real), ...
    numel(omega_real), ...
    size(wheel_real_cmd,1), ...
    size(wheel_real_meas,1), ...
    numel(loss_real), ...
    numel(valid_real), ...
    numel(red_real), ...
    numel(orange_real)]);

t_real = t_real(1:Nreal);
q_real_gt = q_real_gt(1:Nreal,:);
q_real_est = q_real_est(1:Nreal,:);
q_real_ref = q_real_ref(1:Nreal,:);
q_real_vicon = q_real_vicon(1:Nreal,:);
q_real_imu = q_real_imu(1:Nreal,:);
v_real = v_real(1:Nreal);
omega_real = omega_real(1:Nreal);
wheel_real_cmd = wheel_real_cmd(1:Nreal,:);
wheel_real_meas = wheel_real_meas(1:Nreal,:);
loss_real = loss_real(1:Nreal);
valid_real = valid_real(1:Nreal);
red_real = red_real(1:Nreal);
orange_real = orange_real(1:Nreal);

Nsim = min([ ...
    numel(t_sim), ...
    size(q_sim,1), ...
    size(q_sim_est,1), ...
    size(q_sim_ref,1), ...
    size(q_sim_gt,1), ...
    numel(v_sim), ...
    numel(omega_sim), ...
    size(wheel_sim,1), ...
    numel(loss_sim), ...
    numel(red_sim), ...
    numel(orange_sim)]);

t_sim = t_sim(1:Nsim);
q_sim = q_sim(1:Nsim,:);
q_sim_est = q_sim_est(1:Nsim,:);
q_sim_ref = q_sim_ref(1:Nsim,:);
q_sim_gt = q_sim_gt(1:Nsim,:);
v_sim = v_sim(1:Nsim);
omega_sim = omega_sim(1:Nsim);
wheel_sim = wheel_sim(1:Nsim,:);
loss_sim = loss_sim(1:Nsim);
red_sim = red_sim(1:Nsim);
orange_sim = orange_sim(1:Nsim);

%% ============================================================
% WRAP ANGLES
% ============================================================
q_real_gt(:,3) = wrapToPiLocal(q_real_gt(:,3));
q_real_est(:,3) = wrapToPiLocal(q_real_est(:,3));
q_real_ref(:,3) = wrapToPiLocal(q_real_ref(:,3));
q_real_vicon(:,3) = wrapToPiLocal(q_real_vicon(:,3));
q_real_imu(:,3) = wrapToPiLocal(q_real_imu(:,3));

q_sim(:,3) = wrapToPiLocal(q_sim(:,3));
q_sim_est(:,3) = wrapToPiLocal(q_sim_est(:,3));
q_sim_ref(:,3) = wrapToPiLocal(q_sim_ref(:,3));
q_sim_gt(:,3) = wrapToPiLocal(q_sim_gt(:,3));

%% ============================================================
% TIME MARKERS
% ============================================================

% Real transfer start: first time near [1.45, -0.30]
dist_to_task3_start_real = vecnorm(q_real_gt(:,1:2) - task3_start_xy, 2, 2);

idxRealTransfer = find(dist_to_task3_start_real < 0.08, 1, 'first');

if isempty(idxRealTransfer)
    idxRealTransfer = find(q_real_gt(:,1) > 1.40 & abs(q_real_gt(:,2)+0.30) < 0.18, 1, 'first');
end

if isempty(idxRealTransfer)
    idxRealTransfer = 1;
end

t_real_transfer_start = t_real(idxRealTransfer);

% Real stop
motion_mask_real = abs(v_real) > 0.01 | abs(omega_real) > 0.03;

if any(motion_mask_real)
    idxRealStop = find(motion_mask_real, 1, 'last');
else
    idxRealStop = numel(t_real);
end

t_real_motion_end = t_real(idxRealStop);

% Sim circle start
if isfield(Dsim, 'idx_circle_start') && ~isnan(Dsim.idx_circle_start)
    idxSimCircle = round(Dsim.idx_circle_start);
    idxSimCircle = max(1, min(idxSimCircle, numel(t_sim)));
elseif isfield(Dsim, 'task3_active') && any(Dsim.task3_active(:) > 0.5)
    idxSimCircle = find(Dsim.task3_active(:) > 0.5, 1, 'first');
    idxSimCircle = max(1, min(idxSimCircle, numel(t_sim)));
else
    dTargetSim = [0; vecnorm(diff(q_sim_ref(:,1:2),1,1),2,2)];
    idxSimCircle = find(dTargetSim > 1e-5, 1, 'first');

    if isempty(idxSimCircle)
        idxSimCircle = 1;
    end
end

t_sim_circle_start = t_sim(idxSimCircle);

% Sim stop
if isfield(Dsim, 'idx_motion_end') && ~isnan(Dsim.idx_motion_end)
    idxSimStop = round(Dsim.idx_motion_end);
    idxSimStop = max(1, min(idxSimStop, numel(t_sim)));
else
    motion_mask_sim = abs(v_sim) > 0.01 | abs(omega_sim) > 0.03;

    if any(motion_mask_sim)
        idxSimStop = find(motion_mask_sim, 1, 'last');
    else
        idxSimStop = numel(t_sim);
    end
end

t_sim_motion_end = t_sim(idxSimStop);

% Real circle start estimated from real stop and circle duration.
if isfield(Dsim, 'T_circle')
    t_real_circle_start = t_real_motion_end - Dsim.T_circle;
else
    t_real_circle_start = t_real_transfer_start;
end

if t_real_circle_start < t_real_transfer_start
    t_real_circle_start = t_real_transfer_start;
end

[~, idxRealCircle] = min(abs(t_real - t_real_circle_start));
t_real_circle_start = t_real(idxRealCircle);

fprintf('\nTimeline markers:\n');
fprintf('Real transfer/start-point time = %.3f s\n', t_real_transfer_start);
fprintf('Real circle start time         = %.3f s\n', t_real_circle_start);
fprintf('Real stop time                 = %.3f s\n', t_real_motion_end);
fprintf('Sim circle start time          = %.3f s\n', t_sim_circle_start);
fprintf('Sim stop time                  = %.3f s\n', t_sim_motion_end);

%% ============================================================
% TIME AXIS LIMITS FOR REPORT FIGURES
% ============================================================
% The exported simulation may continue after stopping. For report plots,
% show only until shortly after the last stop marker.

tAxisStart = 0.0;
tAxisEnd = ceil(max([t_real_motion_end, t_sim_motion_end]) + 1.0);

fprintf('Time plot x-axis limit         = [%.1f %.1f] s\n', tAxisStart, tAxisEnd);

%% ============================================================
% ERRORS
% ============================================================
real_target_pos_err = vecnorm(q_real_gt(:,1:2) - q_real_ref(:,1:2), 2, 2);
real_target_theta_err = wrapToPiLocal(q_real_gt(:,3) - q_real_ref(:,3));

real_ekf_pos_err = vecnorm(q_real_est(:,1:2) - q_real_gt(:,1:2), 2, 2);
real_ekf_theta_err = wrapToPiLocal(q_real_est(:,3) - q_real_gt(:,3));

if all(isnan(q_real_imu(:)))
    real_imu_pos_err = NaN(size(real_ekf_pos_err));
else
    real_imu_pos_err = vecnorm(q_real_imu(:,1:2) - q_real_gt(:,1:2), 2, 2);
end

real_raw_vicon_pos_err = NaN(size(real_ekf_pos_err));
real_raw_vicon_pos_err(valid_real) = vecnorm( ...
    q_real_vicon(valid_real,1:2) - q_real_gt(valid_real,1:2), 2, 2);

sim_track_pos_err = vecnorm(q_sim(:,1:2) - q_sim_ref(:,1:2), 2, 2);
sim_track_theta_err = wrapToPiLocal(q_sim(:,3) - q_sim_ref(:,3));
sim_ekf_pos_err = vecnorm(q_sim_est(:,1:2) - q_sim_gt(:,1:2), 2, 2);

%% ============================================================
% CIRCLE-ONLY ERRORS
% ============================================================
idxSimCircleRange = idxSimCircle:idxSimStop;
idxRealCircleRange = idxRealCircle:idxRealStop;

sim_circle_track_err = sim_track_pos_err(idxSimCircleRange);
sim_circle_loc_err = sim_ekf_pos_err(idxSimCircleRange);

real_circle_target_err = real_target_pos_err(idxRealCircleRange);
real_circle_loc_err = real_ekf_pos_err(idxRealCircleRange);

%% ============================================================
% LOSS RATES
% ============================================================
realLossIntervals = maskIntervals(t_real, loss_real);

Ts_real = median(diff(t_real));
if isnan(Ts_real) || Ts_real <= 0
    Ts_real = 0.04;
end

lossWindowReal = max(1, round(1.0/Ts_real));
lossRate_real = movmean(double(loss_real), lossWindowReal) * 100;

Ts_sim = median(diff(t_sim));
if isnan(Ts_sim) || Ts_sim <= 0
    Ts_sim = 0.04;
end

lossWindowSim = max(1, round(1.0/Ts_sim));
lossRate_sim = movmean(double(loss_sim), lossWindowSim) * 100;

%% ============================================================
% SMOOTHED SIGNALS FOR PLOTTING ONLY
% ============================================================
v_real_plot = cleanForPlot(v_real, filterWindow, [-0.35 0.35]);
omega_real_plot = cleanForPlot(omega_real, filterWindow, [-3 3]);
wheel_real_cmd_plot = cleanForPlot(wheel_real_cmd, filterWindow, [-11 11]);
wheel_real_meas_plot = cleanForPlot(wheel_real_meas, filterWindow, [-12 12]);

v_sim_plot = cleanForPlot(v_sim, filterWindow, [-0.35 0.35]);
omega_sim_plot = cleanForPlot(omega_sim, filterWindow, [-3 3]);
wheel_sim_plot = cleanForPlot(wheel_sim, filterWindow, [-11 11]);

%% ============================================================
% SUMMARY TABLE
% ============================================================
Summary = table();

Summary.RealDuration_s = t_real(end);
Summary.RealTransferStart_s = t_real_transfer_start;
Summary.RealCircleStart_s = t_real_circle_start;
Summary.RealStop_s = t_real_motion_end;

Summary.SimDuration_s = t_sim(end);
Summary.SimCircleStart_s = t_sim_circle_start;
Summary.SimStop_s = t_sim_motion_end;

Summary.RealFullTargetRMSE_m = sqrt(mean(real_target_pos_err.^2,'omitnan'));
Summary.RealCircleTargetRMSE_m = sqrt(mean(real_circle_target_err.^2,'omitnan'));
Summary.RealEKFRMSE_m = sqrt(mean(real_ekf_pos_err.^2,'omitnan'));
Summary.RealCircleEKFRMSE_m = sqrt(mean(real_circle_loc_err.^2,'omitnan'));
Summary.RealIMURMSE_m = sqrt(mean(real_imu_pos_err.^2,'omitnan'));
Summary.RealRawViconRMSE_m = sqrt(mean(real_raw_vicon_pos_err.^2,'omitnan'));
Summary.RealViconLoss_percent = 100*mean(loss_real,'omitnan');
Summary.RealRed_percent = 100*mean(red_real,'omitnan');
Summary.RealOrange_percent = 100*mean(orange_real,'omitnan');

Summary.SimFullTrackingRMSE_m = sqrt(mean(sim_track_pos_err.^2,'omitnan'));
Summary.SimCircleTrackingRMSE_m = sqrt(mean(sim_circle_track_err.^2,'omitnan'));
Summary.SimFullLocalizationRMSE_m = sqrt(mean(sim_ekf_pos_err.^2,'omitnan'));
Summary.SimCircleLocalizationRMSE_m = sqrt(mean(sim_circle_loc_err.^2,'omitnan'));
Summary.SimViconLoss_percent = 100*mean(loss_sim,'omitnan');

Summary.RealMaxV_m_s = max(abs(v_real),[],'omitnan');
Summary.RealMaxOmega_rad_s = max(abs(omega_real),[],'omitnan');
Summary.RealMaxWheelCmd_rad_s = max(abs(wheel_real_cmd(:)),[],'omitnan');
Summary.RealMaxWheelMeas_rad_s = max(abs(wheel_real_meas(:)),[],'omitnan');

Summary.SimMaxV_m_s = max(abs(v_sim),[],'omitnan');
Summary.SimMaxOmega_rad_s = max(abs(omega_sim),[],'omitnan');
Summary.SimMaxWheelCmd_rad_s = max(abs(wheel_sim(:)),[],'omitnan');

disp('Summary metrics:');
disp(Summary);

writetable(Summary, fullfile(plotsFolder, 'task3_real_vs_sim_summary_metrics.csv'));

%% ============================================================
% FIGURE 1: XY MAP
% ============================================================
fig = newReportFigure(showFigures, figW, figH);
hold on; grid on; axis equal;
drawMap();

pSimRef = plot(q_sim_ref(:,1), q_sim_ref(:,2), 'k--', 'LineWidth', lwRef);
pSim = plot(q_sim(:,1), q_sim(:,2), '-.', 'Color', cSim, 'LineWidth', lwSim);
pSimLoc = plot(q_sim_est(:,1), q_sim_est(:,2), '--', 'Color', cLoc, 'LineWidth', 1.2);

pReal = plot(q_real_gt(:,1), q_real_gt(:,2), '-', 'Color', cVicon, 'LineWidth', lwVicon);
pRealLoc = plot(q_real_est(:,1), q_real_est(:,2), '--', 'Color', cMeas, 'LineWidth', lwMeas);
pRealRef = plot(q_real_ref(:,1), q_real_ref(:,2), ':', 'Color', cRef, 'LineWidth', lwRef);

plot(q_sim(1,1), q_sim(1,2), 'o', ...
    'Color', cSim, 'MarkerFaceColor', cSim, 'MarkerSize', 7, ...
    'HandleVisibility','off');

plot(q_sim(idxSimCircle,1), q_sim(idxSimCircle,2), 's', ...
    'Color', cSim, 'MarkerFaceColor','w', 'MarkerSize', 8, ...
    'LineWidth', 1.5, 'HandleVisibility','off');

plot(q_real_gt(1,1), q_real_gt(1,2), 'o', ...
    'Color', cVicon, 'MarkerFaceColor','w', 'MarkerSize', 7, ...
    'LineWidth', 1.5, 'HandleVisibility','off');

plot(q_real_gt(idxRealCircle,1), q_real_gt(idxRealCircle,2), 's', ...
    'Color', cVicon, 'MarkerFaceColor','w', 'MarkerSize', 8, ...
    'LineWidth', 1.5, 'HandleVisibility','off');

plot(q_real_gt(end,1), q_real_gt(end,2), 'x', ...
    'Color', cVicon, 'MarkerSize', 10, ...
    'LineWidth', 2.0, 'HandleVisibility','off');

xlabel('x [m]');
ylabel('y [m]');
title('Task 3: real robot and simulation from Task 2 end', ...
    'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.055 0]);

legend([pSimRef pSim pSimLoc pReal pRealLoc pRealRef], ...
    {'Simulation reference','Simulation actual','Simulation localizer', ...
     'Real Vicon ground truth','Real localizer','Real target'}, ...
    'Location','eastoutside');

xlim([-0.8 2.95]);
ylim([-1.35 0.85]);

saveReportFigure(fig, plotsFolder, 'task3_01_xy_map_real_vs_sim', fontSize, showFigures);

%% ============================================================
% FIGURE 2: POSE STATES
% ============================================================
fig = newReportFigure(showFigures, figW, figH);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pSimRef = plot(t_sim, q_sim_ref(:,1), 'k--', 'LineWidth', lwRef); hold on;
pSim = plot(t_sim, q_sim(:,1), '-.', 'Color', cSim, 'LineWidth', lwSim);
pReal = plot(t_real, q_real_gt(:,1), '-', 'Color', cVicon, 'LineWidth', lwVicon);
pRealLoc = plot(t_real, q_real_est(:,1), '--', 'Color', cLoc, 'LineWidth', lwLoc);
pRealRef = plot(t_real, q_real_ref(:,1), ':', 'Color', cRef, 'LineWidth', lwRef);
shadeIntervals(gca, realLossIntervals, cMeas, 0.04);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('x [m]');
title('Pose states', 'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
plot(t_sim, q_sim_ref(:,2), 'k--', 'LineWidth', lwRef); hold on;
plot(t_sim, q_sim(:,2), '-.', 'Color', cSim, 'LineWidth', lwSim);
plot(t_real, q_real_gt(:,2), '-', 'Color', cVicon, 'LineWidth', lwVicon);
plot(t_real, q_real_est(:,2), '--', 'Color', cLoc, 'LineWidth', lwLoc);
plot(t_real, q_real_ref(:,2), ':', 'Color', cRef, 'LineWidth', lwRef);
shadeIntervals(gca, realLossIntervals, cMeas, 0.04);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('y [m]');

nexttile;
plot(t_sim, unwrap(q_sim_ref(:,3)), 'k--', 'LineWidth', lwRef); hold on;
plot(t_sim, unwrap(q_sim(:,3)), '-.', 'Color', cSim, 'LineWidth', lwSim);
plot(t_real, unwrap(q_real_gt(:,3)), '-', 'Color', cVicon, 'LineWidth', lwVicon);
plot(t_real, unwrap(q_real_est(:,3)), '--', 'Color', cLoc, 'LineWidth', lwLoc);
plot(t_real, unwrap(q_real_ref(:,3)), ':', 'Color', cRef, 'LineWidth', lwRef);
shadeIntervals(gca, realLossIntervals, cMeas, 0.04);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('\theta [rad]');
xlabel('Time [s]');

lgd = legend(ax1, [pSimRef pSim pReal pRealLoc pRealRef], ...
    {'Sim reference','Sim actual','Real Vicon','Real localizer','Real target'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

setTimeAxes(fig, tAxisStart, tAxisEnd);
saveReportFigure(fig, plotsFolder, 'task3_02_pose_states', fontSize, showFigures);

%% ============================================================
% FIGURE 3: FILTER PERFORMANCE
% ============================================================
fig = newReportFigure(showFigures, figW, figH);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pEKF = plot(t_real, real_ekf_pos_err, '--', 'Color', cLoc, 'LineWidth', lwLoc); hold on;
pRaw = plot(t_real, real_raw_vicon_pos_err, '.', 'Color', cVicon, 'MarkerSize', 7);
shadeIntervals(gca, realLossIntervals, cMeas, 0.04);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on;
ylabel('error [m]');
title('Real EKF/localization performance and Vicon availability', ...
    'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);
ylim([0 0.08]);

nexttile;
pIMU = plot(t_real, real_imu_pos_err, '-.', 'Color', cMeas, 'LineWidth', lwMeas); hold on;
shadeIntervals(gca, realLossIntervals, cMeas, 0.04);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on;
ylabel('odom/IMU error [m]');

nexttile;
plot(t_real, real_target_pos_err, '-', 'Color', cVicon, 'LineWidth', lwVicon); hold on;
shadeIntervals(gca, realLossIntervals, cMeas, 0.04);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on;
ylabel('target error [m]');

nexttile;
pLossRate = plot(t_real, lossRate_real, '-', 'Color', cMeas, 'LineWidth', lwMeas); hold on;
pLossBinary = stairs(t_real, double(loss_real)*100, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 0.8);
pRed = stairs(t_real, double(red_real)*100, '-', 'Color', cRed, 'LineWidth', 1.0);
pOrange = stairs(t_real, double(orange_real)*100, '-', 'Color', cOrange, 'LineWidth', 1.0);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on;
ylabel('flag / loss [%]');
xlabel('Time [s]');
ylim([-5 105]);

lgd = legend(ax1, [pEKF pRaw pIMU pLossRate pLossBinary pRed pOrange], ...
    {'EKF error','Raw Vicon error','Odometry/IMU error', ...
     'Vicon loss rate','Vicon loss flag','Red area','Orange area'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

setTimeAxes(fig, tAxisStart, tAxisEnd);
saveReportFigure(fig, plotsFolder, 'task3_03_filter_performance_loss', fontSize, showFigures);

%% ============================================================
% FIGURE 4: ERRORS REAL VS SIM
% ============================================================
fig = newReportFigure(showFigures, figW, figH);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pSimErr = plot(t_sim, sim_track_pos_err, '-.', 'Color', cSim, 'LineWidth', lwSim); hold on;
pRealErr = plot(t_real, real_target_pos_err, '-', 'Color', cVicon, 'LineWidth', lwVicon);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('target error [m]');
title('Target/tracking and localization errors', ...
    'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
pSimLocErr = plot(t_sim, sim_ekf_pos_err, '-.', 'Color', cSim, 'LineWidth', lwSim); hold on;
pRealLocErr = plot(t_real, real_ekf_pos_err, '--', 'Color', cLoc, 'LineWidth', lwLoc);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('localization error [m]');

nexttile;
pSimLoss = plot(t_sim, lossRate_sim, '-.', 'Color', cSim, 'LineWidth', lwMeas); hold on;
pRealLoss = plot(t_real, lossRate_real, '-', 'Color', cVicon, 'LineWidth', lwMeas);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('Vicon loss [%]');
xlabel('Time [s]');
ylim([-5 105]);

lgd = legend(ax1, [pSimErr pRealErr pSimLocErr pRealLocErr pSimLoss pRealLoss], ...
    {'Sim tracking error','Real target error','Sim localization error','Real EKF error','Sim Vicon loss rate','Real Vicon loss rate'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

setTimeAxes(fig, tAxisStart, tAxisEnd);
saveReportFigure(fig, plotsFolder, 'task3_04_errors_real_vs_sim', fontSize, showFigures);

%% ============================================================
% FIGURE 5: CONTROL INPUTS
% ============================================================
fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pSimV = plot(t_sim, v_sim_plot, '-.', 'Color', cSim, 'LineWidth', lwSim); hold on;
pRealV = plot(t_real, v_real_plot, '-', 'Color', cVicon, 'LineWidth', lwVicon);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('v [m/s]');
title('Control inputs', 'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
pSimW = plot(t_sim, omega_sim_plot, '-.', 'Color', cSim, 'LineWidth', lwSim); hold on;
pRealW = plot(t_real, omega_real_plot, '-', 'Color', cVicon, 'LineWidth', lwVicon);
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('\omega [rad/s]');
xlabel('Time [s]');

lgd = legend(ax1, [pSimV pRealV], {'Simulation command','Real command'}, 'Orientation','horizontal');
lgd.Layout.Tile = 'south';

setTimeAxes(fig, tAxisStart, tAxisEnd);
saveReportFigure(fig, plotsFolder, 'task3_05_control_inputs', fontSize, showFigures);

%% ============================================================
% FIGURE 6: WHEEL SPEEDS
% ============================================================
fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pSimL = plot(t_sim, wheel_sim_plot(:,1), '-.', 'Color', cSim, 'LineWidth', lwSim); hold on;
pRealLCmd = plot(t_real, wheel_real_cmd_plot(:,1), '-', 'Color', cVicon, 'LineWidth', lwVicon);
pRealLMeas = plot(t_real, wheel_real_meas_plot(:,1), '--', 'Color', cMeas, 'LineWidth', lwMeas);
yline( omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');
yline(-omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('\omega_L [rad/s]');
title('Wheel speeds', 'FontWeight','bold', 'Units','normalized', 'Position',[0.5 1.10 0]);

nexttile;
pSimR = plot(t_sim, wheel_sim_plot(:,2), '-.', 'Color', cSim, 'LineWidth', lwSim); hold on;
pRealRCmd = plot(t_real, wheel_real_cmd_plot(:,2), '-', 'Color', cVicon, 'LineWidth', lwVicon);
pRealRMeas = plot(t_real, wheel_real_meas_plot(:,2), '--', 'Color', cMeas, 'LineWidth', lwMeas);
yline( omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');
yline(-omegaLimit, ':', 'Color', cThr, 'LineWidth', lwThr, 'HandleVisibility','off');
addTimelineMarkers(gca, t_real_transfer_start, t_real_circle_start, t_real_motion_end, t_sim_circle_start, t_sim_motion_end);
grid on; ylabel('\omega_R [rad/s]');
xlabel('Time [s]');

lgd = legend(ax1, [pSimL pRealLCmd pRealLMeas], ...
    {'Simulation command','Real command','Real measured'}, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';

setTimeAxes(fig, tAxisStart, tAxisEnd);
saveReportFigure(fig, plotsFolder, 'task3_06_wheel_speeds', fontSize, showFigures);

fprintf('\nAll plots saved in:\n%s\n', fullfile(scriptFolder, plotsFolder));

%% =======================================================================
% LOCAL FUNCTIONS
% =======================================================================

function X = ensureNxM(X, M)
    X = squeeze(X);

    if isvector(X)
        X = X(:).';
    end

    if ndims(X) > 2
        sz = size(X);
        X = reshape(X, [], sz(end));
    end

    if size(X,2) ~= M && size(X,1) == M
        X = X.';
    end

    if size(X,2) ~= M
        error('Expected N x %d, got %s.', M, mat2str(size(X)));
    end
end

function value = getFieldDefault(S, field, defaultValue)
    if isfield(S, field)
        value = S.(field);
    else
        value = defaultValue;
    end
end

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end

function intervals = maskIntervals(t, mask)
    mask = mask(:);

    if isempty(mask) || ~any(mask)
        intervals = zeros(0,2);
        return;
    end

    d = diff([false; mask; false]);
    idxStart = find(d == 1);
    idxEnd = find(d == -1) - 1;

    intervals = [t(idxStart), t(idxEnd)];
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

function addTimelineMarkers(ax, t_real_transfer, t_real_circle, t_real_stop, t_sim_circle, t_sim_stop)
    axes(ax);

    xline(ax, t_real_transfer, ':', 'Real transfer', ...
        'Color', [0.35 0.35 0.35], ...
        'LineWidth', 0.9, ...
        'LabelVerticalAlignment','bottom', ...
        'LabelHorizontalAlignment','left', ...
        'HandleVisibility','off');

    xline(ax, t_real_circle, '--k', 'Real circle', ...
        'LineWidth', 1.0, ...
        'LabelVerticalAlignment','bottom', ...
        'LabelHorizontalAlignment','left', ...
        'HandleVisibility','off');

    xline(ax, t_real_stop, ':k', 'Real stop', ...
        'LineWidth', 0.9, ...
        'LabelVerticalAlignment','bottom', ...
        'LabelHorizontalAlignment','left', ...
        'HandleVisibility','off');

    xline(ax, t_sim_circle, '--', 'Sim circle', ...
        'Color', [0.0 0.45 0.0], ...
        'LineWidth', 1.0, ...
        'LabelVerticalAlignment','top', ...
        'LabelHorizontalAlignment','left', ...
        'HandleVisibility','off');

    xline(ax, t_sim_stop, ':', 'Sim stop', ...
        'Color', [0.0 0.45 0.0], ...
        'LineWidth', 0.9, ...
        'LabelVerticalAlignment','top', ...
        'LabelHorizontalAlignment','left', ...
        'HandleVisibility','off');
end

function setTimeAxes(fig, tStart, tEnd)
    ax = findall(fig, 'Type', 'axes');

    for k = 1:numel(ax)
        try
            xlim(ax(k), [tStart tEnd]);
        catch
        end
    end
end

function drawMap()
    rectangle('Position',[-2.90, -1.50, 5.80, 3.00], ...
        'EdgeColor','k', ...
        'LineWidth',1.2, ...
        'HandleVisibility','off');

    addRect([-2.90,  0.10, 1.40, 1.40], [0.60 0.75 1.00], 0.12);
    addRect([-0.60, -1.50, 0.80, 1.60], [1.00 0.00 0.00], 0.08);
    addRect([ 1.00, -1.50, 1.90, 3.00], [1.00 0.50 0.00], 0.10);

    obstacles = [
        -1.50, -0.70, 0.90, 0.80
         0.20, -1.50, 0.80, 0.80
         1.80, -0.20, 0.40, 0.40
    ];

    for i = 1:size(obstacles,1)
        rectangle('Position', obstacles(i,:), ...
            'FaceColor',[0.55 0.55 0.55], ...
            'EdgeColor','k', ...
            'LineWidth',1.0, ...
            'HandleVisibility','off');
    end
end

function addRect(pos, color, alphaVal)
    x = pos(1);
    y = pos(2);
    w = pos(3);
    h = pos(4);

    patch([x x+w x+w x], ...
          [y y y+h y+h], ...
          color, ...
          'FaceAlpha', alphaVal, ...
          'EdgeColor','none', ...
          'HandleVisibility','off');
end
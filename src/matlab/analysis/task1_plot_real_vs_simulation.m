clear; clc; close all;

%% ============================================================
% Task 1 — Final plotting script
%
% Required files in the same folder:
%   test3_task1_all_outputs.mat
%   exported_task1_simulation_extended_test3.mat
%
% Output folder:
%   ./plots
% ============================================================

%% ---------------- USER SETTINGS ----------------
showFigures = true;

figW = 1100;
figH = 820;
fontSize = 12;

% Robot parameters
r = 0.03;      % wheel radius [m]
d = 0.165;     % wheel distance [m]

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

if showFigures
    set(groot, 'defaultFigureWindowStyle', 'docked');
end

%% ---------------- SCRIPT FOLDER ----------------
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    scriptFolder = pwd;
else
    scriptFolder = fileparts(scriptPath);
end

fprintf('\nScript folder:\n%s\n', scriptFolder);

%% ---------------- FILES ----------------
realFile = fullfile(scriptFolder, 'test3_task1_all_outputs.mat');
simFile  = fullfile(scriptFolder, 'exported_task1_simulation_extended_test3.mat');

plotsFolder = fullfile(scriptFolder, 'plots');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
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

if isfield(Sreal, 'D_task1')
    Dreal = Sreal.D_task1;
else
    error('D_task1 not found inside:\n%s', realFile);
end

if isfield(Dreal, 'Y_viscon_ground_trith')
    q_vicon = ensureNxM(Dreal.Y_viscon_ground_trith, 3);
elseif isfield(Dreal, 'Y_viscon')
    q_vicon = ensureNxM(Dreal.Y_viscon, 3);
else
    error('No Y_viscon_ground_trith or Y_viscon field found in real data.');
end

q_localizer = ensureNxM(Dreal.q_est, 3);
q_target_real = ensureNxM(Dreal.q_target, 3);

t_real = Dreal.t(:);
v_real = Dreal.v(:);
omega_real = Dreal.omega(:);
omega_wheels_real = ensureNxM(Dreal.omega_wheels, 2);

if isfield(Dreal, 'bot_wheelspeed')
    bot_wheelspeed_real = ensureNxM(Dreal.bot_wheelspeed, 2);
else
    error('bot_wheelspeed field not found in real data.');
end

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

q_goal = q_target_real(1,:).';
tRealEnd = t_real(end);

fprintf('\nReal test duration = %.3f s\n', tRealEnd);
fprintf('Goal pose = [%.4f %.4f %.4f]\n', q_goal(1), q_goal(2), q_goal(3));

%% ============================================================
% PLOT-ONLY THETA CORRECTION FOR VICON / LOCALIZER
% ============================================================

q_vicon_plot = q_vicon;
q_localizer_plot = q_localizer;

idx_bad_vicon = findClosestIndex(t_real, 0.72);

if idx_bad_vicon > 1 && idx_bad_vicon < length(t_real)
    rawVal = q_vicon_plot(idx_bad_vicon,3);
    q_vicon_plot(idx_bad_vicon,3) = ...
        0.5 * (q_vicon(idx_bad_vicon-1,3) + q_vicon(idx_bad_vicon+1,3));

    fprintf('\nPlot-only correction: Vicon theta\n');
    fprintf('t = %.3f s | raw = %.4f | corrected = %.4f\n', ...
        t_real(idx_bad_vicon), rawVal, q_vicon_plot(idx_bad_vicon,3));
end

idx_bad_localizer = findClosestIndex(t_real, 0.76);

if idx_bad_localizer > 1 && idx_bad_localizer < length(t_real)
    rawVal = q_localizer_plot(idx_bad_localizer,3);
    q_localizer_plot(idx_bad_localizer,3) = ...
        0.5 * (q_localizer(idx_bad_localizer-1,3) + q_localizer(idx_bad_localizer+1,3));

    fprintf('\nPlot-only correction: Localizer theta\n');
    fprintf('t = %.3f s | raw = %.4f | corrected = %.4f\n', ...
        t_real(idx_bad_localizer), rawVal, q_localizer_plot(idx_bad_localizer,3));
end

%% ============================================================
% PLOT-ONLY COMMAND / WHEEL-SPEED SPIKE CLEANING
% ============================================================

omega_real_plot = omega_real;
omega_wheels_real_plot = omega_wheels_real;
bot_wheelspeed_real_plot = bot_wheelspeed_real;

cmdGlitchWindow = [0.68, 0.84];

omega_real_plot = linearPatchWindow(t_real, omega_real_plot, cmdGlitchWindow);

omega_wheels_real_plot(:,1) = linearPatchWindow(t_real, omega_wheels_real_plot(:,1), cmdGlitchWindow);
omega_wheels_real_plot(:,2) = linearPatchWindow(t_real, omega_wheels_real_plot(:,2), cmdGlitchWindow);

measuredWheelGlitchWindow = [0.40, 1.05];

bot_wheelspeed_real_plot(:,1) = linearPatchWindow(t_real, bot_wheelspeed_real_plot(:,1), measuredWheelGlitchWindow);
bot_wheelspeed_real_plot(:,2) = linearPatchWindow(t_real, bot_wheelspeed_real_plot(:,2), measuredWheelGlitchWindow);

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
    bot_wheelspeed_LR_plot = [bot_wheelspeed_real_plot(:,2), bot_wheelspeed_real_plot(:,1)];
else
    fprintf('Result: measured wheel-speed columns are not swapped. Using original order.\n');
    bot_wheelspeed_LR = bot_wheelspeed_real;
    bot_wheelspeed_LR_plot = bot_wheelspeed_real_plot;
end

%% ============================================================
% RAW WHEEL ODOMETRY FROM MEASURED WHEEL SPEEDS
% No calibration, no Vicon fitting
% ============================================================

q_wheel_odom = reconstructRawWheelOdometry( ...
    t_real, ...
    bot_wheelspeed_LR, ...
    q_vicon_plot(1,:), ...
    r, d);

q_wheel_odom_plot = q_wheel_odom;

%% ============================================================
% LOAD SIMULATION DATA
% ============================================================

Ssim = load(simFile);

if isfield(Ssim, 'Dsim_task1_simulation')
    Dsim = Ssim.Dsim_task1_simulation;
elseif isfield(Ssim, 'Dsim_task1_simulation_extended')
    Dsim = Ssim.Dsim_task1_simulation_extended;
else
    error('Simulation structure not found inside:\n%s', simFile);
end

q_sim = ensureNxM(Dsim.q_actual, 3);

t_sim = Dsim.t(:);
v_sim = Dsim.v(:);
omega_sim = Dsim.omega(:);
omega_wheels_sim = ensureNxM(Dsim.omega_wheels, 2);

Nsim = min([
    length(t_sim)
    size(q_sim,1)
    length(v_sim)
    length(omega_sim)
    size(omega_wheels_sim,1)
]);

t_sim = t_sim(1:Nsim);
q_sim = q_sim(1:Nsim,:);
v_sim = v_sim(1:Nsim);
omega_sim = omega_sim(1:Nsim);
omega_wheels_sim = omega_wheels_sim(1:Nsim,:);

%% ============================================================
% CROP SIMULATION AT CONVERGENCE
% ============================================================

wrap = @(a) atan2(sin(a), cos(a));

e_pos_sim_full = vecnorm(q_sim(:,1:2) - q_goal(1:2).', 2, 2);
e_theta_sim_full = abs(wrap(q_goal(3) - q_sim(:,3)));

posTol = 0.05;
thetaTol = 0.05;

idxConv = find(e_pos_sim_full <= posTol & e_theta_sim_full <= thetaTol, 1, 'first');

if isempty(idxConv)
    idxEndSimPlot = length(t_sim);
    simConvergenceTime = NaN;
    fprintf('\nSimulation did not reach both thresholds in available data.\n');
else
    idxEndSimPlot = idxConv;
    simConvergenceTime = t_sim(idxConv);

    fprintf('\nSimulation convergence time:\n');
    fprintf('t = %.3f s\n', simConvergenceTime);
    fprintf('position error = %.4f m\n', e_pos_sim_full(idxConv));
    fprintf('theta error    = %.4f rad = %.2f deg\n', ...
        e_theta_sim_full(idxConv), rad2deg(e_theta_sim_full(idxConv)));
end

t_sim_plot = t_sim(1:idxEndSimPlot);
q_sim_plot = q_sim(1:idxEndSimPlot,:);
v_sim_plot = v_sim(1:idxEndSimPlot);
omega_sim_plot = omega_sim(1:idxEndSimPlot);
omega_wheels_sim_plot = omega_wheels_sim(1:idxEndSimPlot,:);

e_pos_sim_plot = e_pos_sim_full(1:idxEndSimPlot);
e_theta_sim_plot = e_theta_sim_full(1:idxEndSimPlot);

xMaxPlot = t_sim_plot(end);

%% ============================================================
% ERRORS AND SUMMARY METRICS
% ============================================================

e_pos_vicon = vecnorm(q_vicon_plot(:,1:2) - q_goal(1:2).', 2, 2);
e_theta_vicon = abs(wrap(q_goal(3) - q_vicon_plot(:,3)));

e_pos_localizer = vecnorm(q_localizer_plot(:,1:2) - q_goal(1:2).', 2, 2);
e_theta_localizer = abs(wrap(q_goal(3) - q_localizer_plot(:,3)));

e_pos_wheel_odom = vecnorm(q_wheel_odom_plot(:,1:2) - q_goal(1:2).', 2, 2);
e_theta_wheel_odom = abs(wrap(q_goal(3) - q_wheel_odom_plot(:,3)));

e_localizer_vicon_xy = vecnorm(q_localizer_plot(:,1:2) - q_vicon_plot(:,1:2), 2, 2);
e_localizer_vicon_theta = abs(wrap(q_localizer_plot(:,3) - q_vicon_plot(:,3)));

e_wheel_odom_vicon_xy = vecnorm(q_wheel_odom_plot(:,1:2) - q_vicon_plot(:,1:2), 2, 2);
e_wheel_odom_vicon_theta = abs(wrap(q_wheel_odom_plot(:,3) - q_vicon_plot(:,3)));

vicon_final_pos_err = e_pos_vicon(end);
vicon_final_theta_err = e_theta_vicon(end);

sim_final_pos_err = e_pos_sim_plot(end);
sim_final_theta_err = e_theta_sim_plot(end);

idxSimRealEnd = find(t_sim <= tRealEnd, 1, 'last');

if isempty(idxSimRealEnd)
    idxSimRealEnd = min(length(t_sim), idxEndSimPlot);
end

sim_real_end_pos_err = e_pos_sim_full(idxSimRealEnd);
sim_real_end_theta_err = e_theta_sim_full(idxSimRealEnd);

Summary = table( ...
    tRealEnd, simConvergenceTime, ...
    vicon_final_pos_err, vicon_final_theta_err, rad2deg(vicon_final_theta_err), ...
    sim_real_end_pos_err, sim_real_end_theta_err, rad2deg(sim_real_end_theta_err), ...
    sim_final_pos_err, sim_final_theta_err, rad2deg(sim_final_theta_err), ...
    mean(e_localizer_vicon_xy,'omitnan'), max(e_localizer_vicon_xy,[],'omitnan'), ...
    mean(e_localizer_vicon_theta,'omitnan'), rad2deg(mean(e_localizer_vicon_theta,'omitnan')), ...
    mean(e_wheel_odom_vicon_xy,'omitnan'), max(e_wheel_odom_vicon_xy,[],'omitnan'), ...
    mean(e_wheel_odom_vicon_theta,'omitnan'), rad2deg(mean(e_wheel_odom_vicon_theta,'omitnan')), ...
    max(abs(omega_wheels_real(:)),[],'omitnan'), ...
    max(abs(bot_wheelspeed_real(:)),[],'omitnan'), ...
    max(abs(omega_wheels_sim_plot(:)),[],'omitnan'), ...
    'VariableNames', { ...
    'RealDuration_s','SimConvergenceTime_s', ...
    'ViconFinalPosErr_m','ViconFinalThetaErr_rad','ViconFinalThetaErr_deg', ...
    'SimAtRealEndPosErr_m','SimAtRealEndThetaErr_rad','SimAtRealEndThetaErr_deg', ...
    'SimFinalPlottedPosErr_m','SimFinalPlottedThetaErr_rad','SimFinalPlottedThetaErr_deg', ...
    'MeanLocalizer_Vicon_XY_m','MaxLocalizer_Vicon_XY_m', ...
    'MeanLocalizer_Vicon_Theta_rad','MeanLocalizer_Vicon_Theta_deg', ...
    'MeanWheelOdom_Vicon_XY_m','MaxWheelOdom_Vicon_XY_m', ...
    'MeanWheelOdom_Vicon_Theta_rad','MeanWheelOdom_Vicon_Theta_deg', ...
    'MaxRealWheelCommand_raw','MaxMeasuredWheelSpeed_raw', ...
    'MaxSimWheelCommand_plotted'} ...
);

disp(Summary);

writetable(Summary, fullfile(plotsFolder, 'task1_summary_metrics.csv'));

%% ============================================================
% PLOT STYLE
% ============================================================

cVicon = [0.0000 0.4470 0.7410];
cLoc   = [0.8500 0.3250 0.0980];
cOdom  = [0.3000 0.3000 0.3000];
cSim   = [0.0000 0.5000 0.0000];
cMeas  = [0.4940 0.1840 0.5560];
cThr   = [0.1500 0.1500 0.1500];

lwVicon = 2.3;
lwLoc   = 1.9;
lwOdom  = 1.7;
lwSim   = 2.0;
lwMeas  = 1.5;
lwThr   = 1.1;

%% ============================================================
% FIGURE 1 — XY TRAJECTORY
% ============================================================

fig = newReportFigure(showFigures, figW, figH);

pVicon = plot(q_vicon_plot(:,1), q_vicon_plot(:,2), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pLoc = plot(q_localizer_plot(:,1), q_localizer_plot(:,2), '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

pOdom = plot(q_wheel_odom_plot(:,1), q_wheel_odom_plot(:,2), ':', ...
    'Color', cOdom, 'LineWidth', lwOdom);

pSim = plot(q_sim_plot(:,1), q_sim_plot(:,2), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

pStart = plot(q_vicon_plot(1,1), q_vicon_plot(1,2), 'ko', ...
    'MarkerSize', 8, 'LineWidth', 1.8);

pGoal = plot(q_goal(1), q_goal(2), 'kx', ...
    'MarkerSize', 11, 'LineWidth', 2.2);

grid on;
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('XY Trajectory', 'FontWeight','bold');

legend([pVicon pLoc pOdom pSim pStart pGoal], ...
       {'Vicon', 'Localizer estimate', 'Wheel odometry', 'Simulation', 'Start', 'Goal'}, ...
       'Location','eastoutside');

saveReportFigure(fig, plotsFolder, 'task1_xy_trajectory', fontSize, showFigures);

%% ============================================================
% FIGURE 2 — CONTROL INPUTS
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pReal = plot(t_real, v_real, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pSim = plot(t_sim_plot, v_sim_plot, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('v [m/s]');
title('Control Inputs', 'FontWeight','bold');

nexttile;
plot(t_real, omega_real_plot, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_sim_plot, omega_sim_plot, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('\omega [rad/s]');
xlabel('Time [s]');

lgd = legend(ax1, [pReal pSim], ...
    {'Real command', 'Simulation command'}, ...
    'Orientation','horizontal');

lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task1_control_inputs', fontSize, showFigures);

%% ============================================================
% FIGURE 3 — WHEEL SPEEDS
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pCmd = plot(t_real, omega_wheels_real_plot(:,1), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pMeas = plot(t_real, bot_wheelspeed_LR_plot(:,1), '--', ...
    'Color', cMeas, 'LineWidth', lwMeas);

pSim = plot(t_sim_plot, omega_wheels_sim_plot(:,1), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline(10, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
yline(-10, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('\omega_L [rad/s]');
title('Wheel Speeds', 'FontWeight','bold');

nexttile;
plot(t_real, omega_wheels_real_plot(:,2), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_real, bot_wheelspeed_LR_plot(:,2), '--', ...
    'Color', cMeas, 'LineWidth', lwMeas);

plot(t_sim_plot, omega_wheels_sim_plot(:,2), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline(10, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
yline(-10, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('\omega_R [rad/s]');
xlabel('Time [s]');

lgd = legend(ax1, [pCmd pMeas pSim], ...
    {'Real command', 'Measured wheel speed', 'Simulation command'}, ...
    'Orientation','horizontal');

lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task1_wheel_speeds', fontSize, showFigures);

%% ============================================================
% FIGURE 4 — POSE STATES
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pVicon = plot(t_real, q_vicon_plot(:,1), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pLoc = plot(t_real, q_localizer_plot(:,1), '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

pOdom = plot(t_real, q_wheel_odom_plot(:,1), ':', ...
    'Color', cOdom, 'LineWidth', lwOdom);

pSim = plot(t_sim_plot, q_sim_plot(:,1), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('x [m]');
title('Pose States', 'FontWeight','bold');

nexttile;
plot(t_real, q_vicon_plot(:,2), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_real, q_localizer_plot(:,2), '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

plot(t_real, q_wheel_odom_plot(:,2), ':', ...
    'Color', cOdom, 'LineWidth', lwOdom);

plot(t_sim_plot, q_sim_plot(:,2), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline(q_goal(2), ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('y [m]');

nexttile;
plot(t_real, q_vicon_plot(:,3), '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_real, q_localizer_plot(:,3), '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

plot(t_real, q_wheel_odom_plot(:,3), ':', ...
    'Color', cOdom, 'LineWidth', lwOdom);

plot(t_sim_plot, q_sim_plot(:,3), '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline(q_goal(3), ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('\theta [rad]');
xlabel('Time [s]');

lgd = legend(ax1, [pVicon pLoc pOdom pSim], ...
    {'Vicon', 'Localizer estimate', 'Wheel odometry', 'Simulation'}, ...
    'Orientation','horizontal');

lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task1_pose_states', fontSize, showFigures);

%% ============================================================
% FIGURE 5 — REGULATION ERRORS
% ============================================================

fig = newReportFigure(showFigures, figW, figH);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
pVicon = plot(t_real, e_pos_vicon, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

pLoc = plot(t_real, e_pos_localizer, '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

pOdom = plot(t_real, e_pos_wheel_odom, ':', ...
    'Color', cOdom, 'LineWidth', lwOdom);

pSim = plot(t_sim_plot, e_pos_sim_plot, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline(posTol, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('Position error [m]');
title('Regulation Errors', 'FontWeight','bold');

nexttile;
plot(t_real, e_theta_vicon, '-', ...
    'Color', cVicon, 'LineWidth', lwVicon); hold on;

plot(t_real, e_theta_localizer, '--', ...
    'Color', cLoc, 'LineWidth', lwLoc);

plot(t_real, e_theta_wheel_odom, ':', ...
    'Color', cOdom, 'LineWidth', lwOdom);

plot(t_sim_plot, e_theta_sim_plot, '-.', ...
    'Color', cSim, 'LineWidth', lwSim);

yline(thetaTol, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');
xline(tRealEnd, ':', 'Color', cThr, 'LineWidth', lwThr, ...
    'HandleVisibility','off');

grid on; xlim([0 xMaxPlot]);
ylabel('Orientation error [rad]');
xlabel('Time [s]');

lgd = legend(ax1, [pVicon pLoc pOdom pSim], ...
    {'Vicon', 'Localizer estimate', 'Wheel odometry', 'Simulation'}, ...
    'Orientation','horizontal');

lgd.Layout.Tile = 'south';

saveReportFigure(fig, plotsFolder, 'task1_regulation_errors', fontSize, showFigures);

fprintf('\nAll Task 1 plots saved in:\n%s\n', plotsFolder);

%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function X = ensureNxM(X, M)

    X = squeeze(X);

    if size(X,2) ~= M && size(X,1) == M
        X = X.';
    end

    if size(X,2) ~= M
        error('Signal does not have expected number of columns. Expected %d columns, got size %dx%d.', ...
            M, size(X,1), size(X,2));
    end
end

function idx = findClosestIndex(t, targetTime)

    [~, idx] = min(abs(t(:) - targetTime));
end

function xPatch = linearPatchWindow(t, x, window)

    t = t(:);
    xPatch = x(:);

    idxInside = find(t >= window(1) & t <= window(2));

    if isempty(idxInside)
        return;
    end

    idxBefore = find(t < window(1), 1, 'last');
    idxAfter  = find(t > window(2), 1, 'first');

    if isempty(idxBefore) || isempty(idxAfter)
        return;
    end

    xPatch(idxInside) = interp1( ...
        [t(idxBefore); t(idxAfter)], ...
        [xPatch(idxBefore); xPatch(idxAfter)], ...
        t(idxInside), ...
        'linear');
end

function q_odom = reconstructRawWheelOdometry(t, wheelSpeedLR, q0, r, d)

    t = t(:);
    wheelSpeedLR = ensureNxM(wheelSpeedLR, 2);

    N = min(length(t), size(wheelSpeedLR,1));

    t = t(1:N);
    wheelSpeedLR = wheelSpeedLR(1:N,:);

    omega_L = wheelSpeedLR(:,1);
    omega_R = wheelSpeedLR(:,2);

    v = (r/2) * (omega_R + omega_L);
    omega = (r/d) * (omega_R - omega_L);

    v(~isfinite(v)) = 0;
    omega(~isfinite(omega)) = 0;

    q_odom = integrateOdometry(t, v, omega, q0(:));
end

function q = integrateOdometry(t, v, omega, q0)

    t = t(:);
    v = v(:);
    omega = omega(:);

    N = min([length(t), length(v), length(omega)]);

    t = t(1:N);
    v = v(1:N);
    omega = omega(1:N);

    q = zeros(N,3);
    q(1,:) = q0(:).';

    dtDefault = median(diff(t), 'omitnan');

    if ~isfinite(dtDefault) || dtDefault <= 0
        dtDefault = 0.04;
    end

    for k = 2:N

        Ts_k = t(k) - t(k-1);

        if Ts_k <= 0 || ~isfinite(Ts_k)
            Ts_k = dtDefault;
        end

        theta_prev = q(k-1,3);

        theta_mid = wrapToPiLocal(theta_prev + 0.5 * omega(k-1) * Ts_k);
        theta_new = wrapToPiLocal(theta_prev + omega(k-1) * Ts_k);

        x_new = q(k-1,1) + v(k-1) * cos(theta_mid) * Ts_k;
        y_new = q(k-1,2) + v(k-1) * sin(theta_mid) * Ts_k;

        q(k,:) = [x_new, y_new, theta_new];
    end
end

function a = wrapToPiLocal(a)

    a = atan2(sin(a), cos(a));
end

function fig = newReportFigure(showFigures, figW, figH)

    if showFigures
        fig = figure(100);
        clf(fig);
        set(fig, ...
            'Color','w', ...
            'Position',[80 60 figW figH], ...
            'Visible','on');
    else
        fig = figure('Visible','off', ...
            'Color','w', ...
            'Position',[80 60 figW figH]);
    end
end

function saveReportFigure(fig, plotsFolder, baseName, fontSize, showFigures)

    set(findall(fig,'-property','FontSize'), 'FontSize', fontSize);
    makeFigureWhite(fig);

    drawnow;

    exportgraphics(fig, fullfile(plotsFolder, [baseName '.pdf']), ...
        'ContentType','vector', 'BackgroundColor','white');

    exportgraphics(fig, fullfile(plotsFolder, [baseName '.png']), ...
        'Resolution',300, 'BackgroundColor','white');

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
%% plot_task4_simulink_final_report_style.m
% Final report-style plots for Task 4 Simulink results
% Exact initial point vs non-exact initial point
%
% Output:
% 1) XY trajectory
% 2) pose states
% 3) tracking/localization errors
% 4) control inputs and wheel speeds
% 5) LiDAR innovation and valid beams
%
% The script also writes:
% - task4_final_authoritative_metrics.txt
% - task4_final_authoritative_table.tex

clear; clc; close all;

%% ============================================================
%  Files
% ============================================================

exactFile = "out.simulink.exact.mat";
nonExactFile = "out.simulink.non.exact.mat";

if ~exist(exactFile, "file")
    [f,p] = uigetfile("*.mat", "Select EXACT initial point .mat file");
    exactFile = fullfile(p,f);
end

if ~exist(nonExactFile, "file")
    [f,p] = uigetfile("*.mat", "Select NON-EXACT initial point .mat file");
    nonExactFile = fullfile(p,f);
end

fprintf("Loading exact file: %s\n", exactFile);
fprintf("Loading non-exact file: %s\n", nonExactFile);

exactOut = loadSimOut(exactFile);
nonOut   = loadSimOut(nonExactFile);

exact = packOutput(exactOut, "Exact initial point");
nonExact = packOutput(nonOut, "Non-exact initial point");

%% ============================================================
%  Output folder and style
% ============================================================

outDir = "task4_simulink_final_report_plots";

if ~exist(outDir, "dir")
    mkdir(outDir);
end

setDefaultPlotStyle();
style = getPlotStyle();

% Time range shown in report plots
plotTimeLim = [0 40];

%% ============================================================
%  Compute errors and metrics
% ============================================================

exact = computeErrorsAndMetrics(exact, plotTimeLim);
nonExact = computeErrorsAndMetrics(nonExact, plotTimeLim);

%% ============================================================
%  Figure 1: XY trajectory
% ============================================================

fig = figure('Name','XY trajectory', 'Color','w');
fig.Position = [80 80 style.figW style.figH];

hold on; grid on; axis equal;
styleAxes(gca, style);

drawCube(style);

hRef = plot(exact.q_target(:,1), exact.q_target(:,2), ...
    'k--', 'LineWidth', 1.9);

hExactTrue = plot(exact.q_actual(:,1), exact.q_actual(:,2), ...
    '-.', 'Color', style.cExactTrue, 'LineWidth', 1.8);

hExactEKF = plot(exact.q_est(:,1), exact.q_est(:,2), ...
    '--', 'Color', style.cExactEKF, 'LineWidth', 1.7);

hNonTrue = plot(nonExact.q_actual(:,1), nonExact.q_actual(:,2), ...
    '-', 'Color', style.cNonTrue, 'LineWidth', 1.8);

hNonEKF = plot(nonExact.q_est(:,1), nonExact.q_est(:,2), ...
    ':', 'Color', style.cNonEKF, 'LineWidth', 2.1);

% Exact initial point
hStartExact = plot(exact.q_actual(1,1), exact.q_actual(1,2), 'o', ...
    'MarkerFaceColor', style.cExactTrue, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'MarkerSize', 8);

% Non-exact initial point
hStartNonExact = plot(nonExact.q_actual(1,1), nonExact.q_actual(1,2), 's', ...
    'MarkerFaceColor', style.cNonTrue, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'MarkerSize', 8);

% Circle start point
hCircleStart = plot(1.45, -0.30, 'd', ...
    'MarkerFaceColor', style.cCircleStart, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'MarkerSize', 8);

% Final point
idxEndNon = nonExact.m.idxFinal;
hEnd = plot(nonExact.q_actual(idxEndNon,1), nonExact.q_actual(idxEndNon,2), 'o', ...
    'MarkerFaceColor', style.cEnd, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'MarkerSize', 8);

xlabel('x [m]');
ylabel('y [m]');
title('XY trajectory - exact vs non-exact initial point');

lgd = legend([hRef hExactTrue hExactEKF hNonTrue hNonEKF ...
              hStartExact hStartNonExact hCircleStart hEnd], ...
    {'Reference', ...
     'Exact true', 'Exact EKF', ...
     'Non-exact true', 'Non-exact EKF', ...
     'Exact start', 'Non-exact start', 'Circle start', 'End'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 3;
styleLegend(lgd);

setXYLimits(exact, nonExact);

saveFigure(outDir, "task4_simulink_01_xy_map_exact_vs_non_exact");

%% ============================================================
%  Figure 2: Pose states
% ============================================================

fig = figure('Name','States', 'Color','w');
fig.Position = [100 100 style.figW style.figH];

thetaRef = unwrap(exact.q_target(:,3));
thetaExactTrue = unwrap(exact.q_actual(:,3));
thetaExactEKF = unwrap(exact.q_est(:,3));
thetaNonTrue = unwrap(nonExact.q_actual(:,3));
thetaNonEKF = unwrap(nonExact.q_est(:,3));

tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

h1 = plot(exact.t, exact.q_target(:,1), 'k--', 'LineWidth', 1.6);
h2 = plot(exact.t, exact.q_actual(:,1), '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
h3 = plot(exact.t, exact.q_est(:,1), '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
h4 = plot(nonExact.t, nonExact.q_actual(:,1), '-', 'Color', style.cNonTrue, 'LineWidth', 1.5);
h5 = plot(nonExact.t, nonExact.q_est(:,1), ':', 'Color', style.cNonEKF, 'LineWidth', 1.9);

ylabel('x [m]');
title('Longitudinal position');

lgd = legend([h1 h2 h3 h4 h5], ...
    {'Reference','Exact true','Exact EKF','Non-exact true','Non-exact EKF'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 5;
styleLegend(lgd);

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

plot(exact.t, exact.q_target(:,2), 'k--', 'LineWidth', 1.6);
plot(exact.t, exact.q_actual(:,2), '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
plot(exact.t, exact.q_est(:,2), '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
plot(nonExact.t, nonExact.q_actual(:,2), '-', 'Color', style.cNonTrue, 'LineWidth', 1.5);
plot(nonExact.t, nonExact.q_est(:,2), ':', 'Color', style.cNonEKF, 'LineWidth', 1.9);

ylabel('y [m]');
title('Lateral position');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

plot(exact.t, thetaRef, 'k--', 'LineWidth', 1.6);
plot(exact.t, thetaExactTrue, '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
plot(exact.t, thetaExactEKF, '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
plot(nonExact.t, thetaNonTrue, '-', 'Color', style.cNonTrue, 'LineWidth', 1.5);
plot(nonExact.t, thetaNonEKF, ':', 'Color', style.cNonEKF, 'LineWidth', 1.9);

ylabel('\theta [rad]');
xlabel('time [s]');
title('Orientation, unwrapped for visualization');

saveFigure(outDir, "task4_simulink_02_states_exact_vs_non_exact");

%% ============================================================
%  Figure 3: Errors
% ============================================================

fig = figure('Name','Errors', 'Color','w');
fig.Position = [120 120 style.figW style.figH];

tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

h1 = plot(exact.t, exact.eTrackXY, '-', ...
    'Color', style.cExactTrue, 'LineWidth', 1.7);

h2 = plot(exact.t, exact.eLocXY, '--', ...
    'Color', style.cExactEKF, 'LineWidth', 1.7);

h3 = plot(nonExact.t, nonExact.eTrackXY, '-', ...
    'Color', style.cNonTrue, 'LineWidth', 1.7);

h4 = plot(nonExact.t, nonExact.eLocXY, '--', ...
    'Color', style.cNonEKF, 'LineWidth', 1.7);

ylabel('position error [m]');
title('Position errors');

lgd = legend([h1 h2 h3 h4], ...
    {'Exact tracking','Exact localization', ...
     'Non-exact tracking','Non-exact localization'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 4;
styleLegend(lgd);

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

plot(exact.t, rad2deg(exact.eTrackTh), '-', ...
    'Color', style.cExactTrue, 'LineWidth', 1.7);

plot(exact.t, rad2deg(exact.eLocTh), '--', ...
    'Color', style.cExactEKF, 'LineWidth', 1.7);

plot(nonExact.t, rad2deg(nonExact.eTrackTh), '-', ...
    'Color', style.cNonTrue, 'LineWidth', 1.7);

plot(nonExact.t, rad2deg(nonExact.eLocTh), '--', ...
    'Color', style.cNonEKF, 'LineWidth', 1.7);

ylabel('angle error [deg]');
xlabel('time [s]');
title('Orientation errors');

saveFigure(outDir, "task4_simulink_03_errors_exact_vs_non_exact");

%% ============================================================
%  Figure 4: Inputs
% ============================================================

fig = figure('Name','Inputs', 'Color','w');
fig.Position = [140 140 style.figW style.figH];

tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

h1 = plot(exact.t, exact.v, '-', ...
    'Color', style.cExactTrue, 'LineWidth', 1.6);

h2 = plot(exact.t, exact.omega, '--', ...
    'Color', style.cExactEKF, 'LineWidth', 1.6);

h3 = plot(nonExact.t, nonExact.v, '-', ...
    'Color', style.cNonTrue, 'LineWidth', 1.6);

h4 = plot(nonExact.t, nonExact.omega, '--', ...
    'Color', style.cNonEKF, 'LineWidth', 1.6);

ylabel('control input');
title('Kinematic control inputs');

lgd = legend([h1 h2 h3 h4], ...
    {'Exact v [m/s]','Exact \omega [rad/s]', ...
     'Non-exact v [m/s]','Non-exact \omega [rad/s]'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 4;
styleLegend(lgd);

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

wExact = makeNx2(exact.omega_wheels);
wNon = makeNx2(nonExact.omega_wheels);

NwE = min(size(wExact,1), numel(exact.t));
NwN = min(size(wNon,1), numel(nonExact.t));

h1 = plot(exact.t(1:NwE), wExact(1:NwE,1), '-', ...
    'Color', style.cExactTrue, 'LineWidth', 1.6);

h2 = plot(exact.t(1:NwE), wExact(1:NwE,2), '--', ...
    'Color', style.cExactEKF, 'LineWidth', 1.6);

h3 = plot(nonExact.t(1:NwN), wNon(1:NwN,1), '-', ...
    'Color', style.cNonTrue, 'LineWidth', 1.6);

h4 = plot(nonExact.t(1:NwN), wNon(1:NwN,2), '--', ...
    'Color', style.cNonEKF, 'LineWidth', 1.6);

yline(10, 'k:', 'LineWidth', 1.1, 'HandleVisibility','off');
yline(-10, 'k:', 'LineWidth', 1.1, 'HandleVisibility','off');

ylabel('wheel speed [rad/s]');
xlabel('time [s]');
title('Wheel speed commands');

lgd = legend([h1 h2 h3 h4], ...
    {'Exact \omega_L','Exact \omega_R', ...
     'Non-exact \omega_L','Non-exact \omega_R'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 4;
styleLegend(lgd);

saveFigure(outDir, "task4_simulink_04_inputs_exact_vs_non_exact");

%% ============================================================
%  Figure 5: LiDAR innovation
% ============================================================

fig = figure('Name','LiDAR innovation', 'Color','w');
fig.Position = [160 160 style.figW style.figH];

tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

h1 = plot(exact.t, exact.innovation_RMS, '-', ...
    'Color', style.cExactEKF, 'LineWidth', 1.5);

h2 = plot(nonExact.t, nonExact.innovation_RMS, '--', ...
    'Color', style.cNonEKF, 'LineWidth', 1.5);

ylabel('RMS innovation [m]');
title('LiDAR innovation after gating');

lgd = legend([h1 h2], ...
    {'Exact initial point','Non-exact initial point'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 2;
styleLegend(lgd);

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

h1 = plot(exact.t, exact.n_valid_lidar, '-', ...
    'Color', style.cExactEKF, 'LineWidth', 1.5);

h2 = plot(nonExact.t, nonExact.n_valid_lidar, '--', ...
    'Color', style.cNonEKF, 'LineWidth', 1.5);

ylabel('valid LiDAR rays');
xlabel('time [s]');
title('Number of LiDAR rays used in EKF update');

lgd = legend([h1 h2], ...
    {'Exact initial point','Non-exact initial point'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 2;
styleLegend(lgd);

saveFigure(outDir, "task4_simulink_05_lidar_innovation_exact_vs_non_exact");

%% ============================================================
%  Numerical reports
% ============================================================

writeTextReport(fullfile(outDir, "task4_final_authoritative_metrics.txt"), exact, nonExact);
writeLatexTable(fullfile(outDir, "task4_final_authoritative_table.tex"), exact, nonExact);

printMetrics(exact);
printMetrics(nonExact);

fprintf('\nFigures and reports saved in:\n%s\n\n', outDir);

%% ========================================================================
%                               FUNCTIONS
% ========================================================================

function setDefaultPlotStyle()

    set(groot, 'defaultFigureColor', 'w');
    set(groot, 'defaultAxesColor', 'w');
    set(groot, 'defaultAxesXColor', 'k');
    set(groot, 'defaultAxesYColor', 'k');
    set(groot, 'defaultTextColor', 'k');
    set(groot, 'defaultAxesFontName', 'Times New Roman');
    set(groot, 'defaultTextFontName', 'Times New Roman');
    set(groot, 'defaultAxesFontSize', 12);
    set(groot, 'defaultLineLineWidth', 1.4);

end

function style = getPlotStyle()

    style.cRef = [0.0000 0.0000 0.0000];

    style.cExactTrue = [0.0000 0.5000 0.0000];
    style.cExactEKF  = [0.8500 0.3250 0.0980];

    style.cNonTrue   = [0.0000 0.4470 0.7410];
    style.cNonEKF    = [0.4940 0.1840 0.5560];

    style.cStart = [0.0000 0.6000 0.0000];
    style.cCircleStart = [0.9290 0.6940 0.1250];
    style.cEnd   = [0.6000 0.0000 0.6000];

    style.gridColor = [0.78 0.78 0.78];

    style.cubeFace = [0.86 0.86 0.86];
    style.cubeEdge = [0.00 0.00 0.00];

    % Taller figures because legends are below the plots
    style.figW = 1150;
    style.figH = 920;

end

function styleAxes(ax, style)

    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = style.gridColor;
    ax.GridAlpha = 0.45;
    ax.LineWidth = 0.8;
    ax.FontName = 'Times New Roman';
    ax.FontSize = 12;
    ax.Box = 'on';

end

function styleLegend(lgd)

    lgd.Color = 'w';
    lgd.TextColor = 'k';
    lgd.EdgeColor = [0.75 0.75 0.75];
    lgd.FontName = 'Times New Roman';
    lgd.FontSize = 10;
    lgd.Orientation = 'horizontal';

end

function simOut = loadSimOut(matFile)

    S = load(matFile);

    if isfield(S, "out")
        simOut = S.out;
        return;
    end

    names = fieldnames(S);

    for i = 1:numel(names)
        obj = S.(names{i});
        if isa(obj, "Simulink.SimulationOutput")
            simOut = obj;
            return;
        end
    end

    error("No Simulink output object named 'out' was found in %s.", matFile);

end

function R = packOutput(simOut, caseName)

    R = struct();
    R.caseName = string(caseName);

    R.q_target = makeNx3(getSigData(simOut.q_target));
    R.q_actual = makeNx3(getSigData(simOut.q_actual));
    R.q_est    = makeNx3(getSigData(simOut.q_est));

    R.t = getTime(simOut, R.q_est);
    R.t = R.t(:);

    R.v = getSigData(simOut.v);
    R.v = R.v(:);

    R.omega = getSigData(simOut.omega);
    R.omega = R.omega(:);

    R.omega_wheels = makeNx2(getSigData(simOut.omega_wheels));

    if isprop(simOut, "innovation_RMS")
        R.innovation_RMS = getSigData(simOut.innovation_RMS);
    else
        R.innovation_RMS = [];
    end

    if isprop(simOut, "n_valid_lidar_sim")
        R.n_valid_lidar = getSigData(simOut.n_valid_lidar_sim);
    elseif isprop(simOut, "n_valid_lidar")
        R.n_valid_lidar = getSigData(simOut.n_valid_lidar);
    else
        R.n_valid_lidar = [];
    end

    if ~isempty(R.innovation_RMS)
        R.innovation_RMS = R.innovation_RMS(:);
    end

    if ~isempty(R.n_valid_lidar)
        R.n_valid_lidar = R.n_valid_lidar(:);
    end

    N = min([ ...
        numel(R.t), ...
        size(R.q_target,1), ...
        size(R.q_actual,1), ...
        size(R.q_est,1), ...
        numel(R.v), ...
        numel(R.omega), ...
        size(R.omega_wheels,1)]);

    if ~isempty(R.innovation_RMS)
        N = min(N, numel(R.innovation_RMS));
    end

    if ~isempty(R.n_valid_lidar)
        N = min(N, numel(R.n_valid_lidar));
    end

    R.t = R.t(1:N);
    R.q_target = R.q_target(1:N,:);
    R.q_actual = R.q_actual(1:N,:);
    R.q_est = R.q_est(1:N,:);
    R.v = R.v(1:N);
    R.omega = R.omega(1:N);
    R.omega_wheels = R.omega_wheels(1:N,:);

    if ~isempty(R.innovation_RMS)
        R.innovation_RMS = R.innovation_RMS(1:N);
    end

    if ~isempty(R.n_valid_lidar)
        R.n_valid_lidar = R.n_valid_lidar(1:N);
    end

end

function X = getSigData(sig)

    if isa(sig, 'timeseries')
        X = sig.Data;

    elseif isstruct(sig)

        if isfield(sig, 'signals')
            X = sig.signals.values;
        elseif isfield(sig, 'Data')
            X = sig.Data;
        elseif isfield(sig, 'data')
            X = sig.data;
        else
            X = sig;
        end

    else
        X = sig;
    end

    X = squeeze(X);

end

function t = getTime(simOut, qEst)

    if isprop(simOut, "tout")
        t = simOut.tout;
    elseif isprop(simOut, "q_est") && isa(simOut.q_est, "timeseries")
        t = simOut.q_est.Time;
    else
        t = (0:size(qEst,1)-1)'*0.04;
    end

end

function X = makeNx3(X)

    X = squeeze(X);

    if size(X,2) ~= 3 && size(X,1) == 3
        X = X';
    end

end

function W = makeNx2(W)

    W = squeeze(W);

    if size(W,2) ~= 2 && size(W,1) == 2
        W = W';
    end

end

function R = computeErrorsAndMetrics(R, plotTimeLim)

    R.eTrackXY = sqrt(sum((R.q_actual(:,1:2) - R.q_target(:,1:2)).^2, 2));
    R.eLocXY   = sqrt(sum((R.q_est(:,1:2) - R.q_actual(:,1:2)).^2, 2));

    R.eTrackTh = wrapPiLocal(R.q_target(:,3) - R.q_actual(:,3));
    R.eLocTh   = wrapPiLocal(R.q_est(:,3) - R.q_actual(:,3));

    idx = R.t >= plotTimeLim(1) & R.t <= plotTimeLim(2);
    idxFinal = find(idx, 1, "last");

    R.m.idx = idx;
    R.m.idxFinal = idxFinal;

    R.m.duration = R.t(idxFinal);

    R.m.initial_track_xy = R.eTrackXY(1);
    R.m.initial_loc_xy = R.eLocXY(1);

    R.m.initial_track_heading = rad2deg(R.eTrackTh(1));
    R.m.initial_loc_heading = rad2deg(R.eLocTh(1));

    R.m.track_rmse_xy = sqrt(mean(R.eTrackXY(idx).^2));
    R.m.track_max_xy = max(R.eTrackXY(idx));
    R.m.track_final_xy = R.eTrackXY(idxFinal);

    R.m.loc_rmse_xy = sqrt(mean(R.eLocXY(idx).^2));
    R.m.loc_max_xy = max(R.eLocXY(idx));
    R.m.loc_final_xy = R.eLocXY(idxFinal);

    R.m.track_rmse_heading = rad2deg(sqrt(mean(R.eTrackTh(idx).^2)));
    R.m.loc_rmse_heading = rad2deg(sqrt(mean(R.eLocTh(idx).^2)));

    R.m.max_track_heading = max(abs(rad2deg(R.eTrackTh(idx))));
    R.m.max_loc_heading = max(abs(rad2deg(R.eLocTh(idx))));

    R.m.max_abs_v = max(abs(R.v(idx)));
    R.m.max_abs_omega = max(abs(R.omega(idx)));

    R.m.max_abs_wheel_speed = max(abs(R.omega_wheels(idx,:)), [], "all");

    if ~isempty(R.innovation_RMS)
        R.m.mean_innovation = mean(R.innovation_RMS(idx), "omitnan");
        R.m.max_innovation = max(R.innovation_RMS(idx), [], "omitnan");
        R.m.valid_update_percent = 100 * mean(~isnan(R.innovation_RMS(idx)));
        R.m.skipped_update_percent = 100 - R.m.valid_update_percent;
    else
        R.m.mean_innovation = NaN;
        R.m.max_innovation = NaN;
        R.m.valid_update_percent = NaN;
        R.m.skipped_update_percent = NaN;
    end

    if ~isempty(R.n_valid_lidar)
        R.m.mean_valid_rays = mean(R.n_valid_lidar(idx), "omitnan");
        R.m.min_valid_rays = min(R.n_valid_lidar(idx), [], "omitnan");
        R.m.max_valid_rays = max(R.n_valid_lidar(idx), [], "omitnan");
    else
        R.m.mean_valid_rays = NaN;
        R.m.min_valid_rays = NaN;
        R.m.max_valid_rays = NaN;
    end

end

function printMetrics(R)

    fprintf('\nMetrics for %s:\n', R.caseName);
    fprintf('  Trajectory duration          = %.2f s\n', R.m.duration);
    fprintf('  Initial tracking position    = %.4f m\n', R.m.initial_track_xy);
    fprintf('  Initial localization position= %.4f m\n', R.m.initial_loc_xy);
    fprintf('  Tracking RMSE position       = %.4f m\n', R.m.track_rmse_xy);
    fprintf('  Tracking max position error  = %.4f m\n', R.m.track_max_xy);
    fprintf('  Tracking RMSE heading        = %.2f deg\n', R.m.track_rmse_heading);
    fprintf('  Localization RMSE position   = %.4f m\n', R.m.loc_rmse_xy);
    fprintf('  Localization max position    = %.4f m\n', R.m.loc_max_xy);
    fprintf('  Localization final position  = %.4f m\n', R.m.loc_final_xy);
    fprintf('  Localization RMSE heading    = %.2f deg\n', R.m.loc_rmse_heading);
    fprintf('  Max localization heading     = %.2f deg\n', R.m.max_loc_heading);
    fprintf('  Max wheel speed              = %.4f rad/s\n', R.m.max_abs_wheel_speed);
    fprintf('  Mean LiDAR innovation        = %.4f m\n', R.m.mean_innovation);
    fprintf('  Max LiDAR innovation         = %.4f m\n', R.m.max_innovation);
    fprintf('  Mean valid LiDAR rays        = %.2f\n', R.m.mean_valid_rays);
    fprintf('  Valid LiDAR update percent   = %.2f %%\n', R.m.valid_update_percent);

end

function writeTextReport(fileName, exact, nonExact)

    fid = fopen(fileName, "w");

    writeOneText(fid, exact);
    fprintf(fid, "\n");
    writeOneText(fid, nonExact);

    fclose(fid);

end

function writeOneText(fid, R)

    fprintf(fid, "Metrics for %s:\n", R.caseName);
    fprintf(fid, "Trajectory duration [s] = %.2f\n", R.m.duration);
    fprintf(fid, "Initial tracking position error [m] = %.4f\n", R.m.initial_track_xy);
    fprintf(fid, "Initial localization position error [m] = %.4f\n", R.m.initial_loc_xy);
    fprintf(fid, "Initial tracking heading error [deg] = %.2f\n", R.m.initial_track_heading);
    fprintf(fid, "Initial localization heading error [deg] = %.2f\n", R.m.initial_loc_heading);
    fprintf(fid, "Tracking RMSE position [m] = %.4f\n", R.m.track_rmse_xy);
    fprintf(fid, "Tracking max position error [m] = %.4f\n", R.m.track_max_xy);
    fprintf(fid, "Tracking final position error [m] = %.4f\n", R.m.track_final_xy);
    fprintf(fid, "Tracking RMSE heading [deg] = %.2f\n", R.m.track_rmse_heading);
    fprintf(fid, "Localization RMSE position [m] = %.4f\n", R.m.loc_rmse_xy);
    fprintf(fid, "Localization max position error [m] = %.4f\n", R.m.loc_max_xy);
    fprintf(fid, "Localization final position error [m] = %.4f\n", R.m.loc_final_xy);
    fprintf(fid, "Localization RMSE heading [deg] = %.2f\n", R.m.loc_rmse_heading);
    fprintf(fid, "Maximum localization heading error [deg] = %.2f\n", R.m.max_loc_heading);
    fprintf(fid, "Maximum |v| [m/s] = %.4f\n", R.m.max_abs_v);
    fprintf(fid, "Maximum |omega| [rad/s] = %.4f\n", R.m.max_abs_omega);
    fprintf(fid, "Maximum commanded wheel speed [rad/s] = %.4f\n", R.m.max_abs_wheel_speed);
    fprintf(fid, "Mean LiDAR innovation [m] = %.4f\n", R.m.mean_innovation);
    fprintf(fid, "Max LiDAR innovation [m] = %.4f\n", R.m.max_innovation);
    fprintf(fid, "Mean valid LiDAR rays = %.2f\n", R.m.mean_valid_rays);
    fprintf(fid, "Min/Max valid LiDAR rays = %.0f / %.0f\n", R.m.min_valid_rays, R.m.max_valid_rays);
    fprintf(fid, "Valid LiDAR update percentage [%%] = %.2f\n", R.m.valid_update_percent);
    fprintf(fid, "Skipped LiDAR update percentage [%%] = %.2f\n", R.m.skipped_update_percent);

end

function writeLatexTable(fileName, exact, nonExact)

    fid = fopen(fileName, "w");

    fprintf(fid, "\\begin{table}[H]\n");
    fprintf(fid, "\\centering\n");
    fprintf(fid, "\\caption{Numerical summary of Task 4 simulation results.}\n");
    fprintf(fid, "\\label{tab:task4_results}\n");
    fprintf(fid, "\\begin{tabular}{lll}\n");
    fprintf(fid, "\\hline\n");
    fprintf(fid, "Metric & Exact initial point & Non-exact initial point \\\\\n");
    fprintf(fid, "\\hline\n");
    fprintf(fid, "Trajectory duration [s] & %.2f & %.2f \\\\\n", exact.m.duration, nonExact.m.duration);
    fprintf(fid, "Tracking RMSE position [m] & %.4f & %.4f \\\\\n", exact.m.track_rmse_xy, nonExact.m.track_rmse_xy);
    fprintf(fid, "Tracking max position error [m] & %.4f & %.4f \\\\\n", exact.m.track_max_xy, nonExact.m.track_max_xy);
    fprintf(fid, "Localization RMSE position [m] & %.4f & %.4f \\\\\n", exact.m.loc_rmse_xy, nonExact.m.loc_rmse_xy);
    fprintf(fid, "Maximum localization error [m] & %.4f & %.4f \\\\\n", exact.m.loc_max_xy, nonExact.m.loc_max_xy);
    fprintf(fid, "Final localization error [m] & %.4f & %.4f \\\\\n", exact.m.loc_final_xy, nonExact.m.loc_final_xy);
    fprintf(fid, "Maximum localization heading error [deg] & %.2f & %.2f \\\\\n", exact.m.max_loc_heading, nonExact.m.max_loc_heading);
    fprintf(fid, "Maximum commanded wheel speed [rad/s] & %.4f & %.4f \\\\\n", exact.m.max_abs_wheel_speed, nonExact.m.max_abs_wheel_speed);
    fprintf(fid, "Mean LiDAR innovation [m] & %.4f & %.4f \\\\\n", exact.m.mean_innovation, nonExact.m.mean_innovation);
    fprintf(fid, "Mean valid LiDAR rays & %.2f & %.2f \\\\\n", exact.m.mean_valid_rays, nonExact.m.mean_valid_rays);
    fprintf(fid, "Valid LiDAR update percentage [\\%%] & %.2f & %.2f \\\\\n", exact.m.valid_update_percent, nonExact.m.valid_update_percent);
    fprintf(fid, "\\hline\n");
    fprintf(fid, "\\end{tabular}\n");
    fprintf(fid, "\\end{table}\n");

    fclose(fid);

end

function drawCube(style)

    rectangle('Position',[1.80 -0.20 0.40 0.40], ...
        'FaceColor', style.cubeFace, ...
        'EdgeColor', style.cubeEdge, ...
        'LineWidth',2, ...
        'HandleVisibility','off');

    plot(2.00, 0.00, 'kx', ...
        'LineWidth', 2, ...
        'MarkerSize', 10, ...
        'HandleVisibility','off');

end

function setXYLimits(exact, nonExact)

    allX = [exact.q_target(:,1); exact.q_actual(:,1); exact.q_est(:,1); ...
            nonExact.q_actual(:,1); nonExact.q_est(:,1); 1.80; 2.20];

    allY = [exact.q_target(:,2); exact.q_actual(:,2); exact.q_est(:,2); ...
            nonExact.q_actual(:,2); nonExact.q_est(:,2); -0.20; 0.20];

    xlim([min(allX)-0.15, max(allX)+0.15]);
    ylim([min(allY)-0.15, max(allY)+0.15]);

end

function saveFigure(outDir, name)

    pdfName = fullfile(outDir, name + ".pdf");
    pngName = fullfile(outDir, name + ".png");

    exportgraphics(gcf, pdfName, 'ContentType','vector');
    exportgraphics(gcf, pngName, 'Resolution',300);

end

function a = wrapPiLocal(a)

    a = atan2(sin(a), cos(a));

end
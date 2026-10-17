%% plot_task4_exact_vs_wrong_ekf.m
% Task 4 final plots and metrics:
% Exact EKF initialization vs wrong EKF initialization
%
% Required files:
%   out.simulink.exact.mat
%   out.simulink.wrong_ekf.mat

clear; clc; close all;

%% ============================================================
% Files
% ============================================================
exactFile = "out.simulink.exact_analyticH.mat";
wrongFile = "out.simulink.wrong_ekf_analyticH.mat";

if ~exist(exactFile, "file")
    [f,p] = uigetfile("*.mat", "Select EXACT initialization .mat file");
    exactFile = fullfile(p,f);
end

if ~exist(wrongFile, "file")
    [f,p] = uigetfile("*.mat", "Select WRONG EKF initialization .mat file");
    wrongFile = fullfile(p,f);
end

exactOut = loadSimOut(exactFile);
wrongOut = loadSimOut(wrongFile);

exact = packOutput(exactOut, "Exact initialization");
wrong = packOutput(wrongOut, "Wrong EKF initialization");

%% ============================================================
% Output folder and style
% ============================================================

outDir = "task4_exact_vs_wrong_ekf_analyticH_plots";

if ~exist(outDir, "dir")
    mkdir(outDir);
end

setDefaultPlotStyle();
style = getPlotStyle();

plotTimeLim = [0 40];

%% ============================================================
% Compute errors and metrics
% ============================================================

exact = computeErrorsAndMetrics(exact, plotTimeLim);
wrong = computeErrorsAndMetrics(wrong, plotTimeLim);

%% ============================================================
% Figure 1: XY trajectory
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

hWrongTrue = plot(wrong.q_actual(:,1), wrong.q_actual(:,2), ...
    '-', 'Color', style.cWrongTrue, 'LineWidth', 1.8);

hWrongEKF = plot(wrong.q_est(:,1), wrong.q_est(:,2), ...
    ':', 'Color', style.cWrongEKF, 'LineWidth', 2.1);

% True robot initial point
hTrueStart = plot(wrong.q_actual(1,1), wrong.q_actual(1,2), 'o', ...
    'MarkerFaceColor', style.cStart, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'MarkerSize', 8);

% Wrong EKF initial estimate
hEKFStart = plot(wrong.q_est(1,1), wrong.q_est(1,2), 's', ...
    'MarkerFaceColor', style.cWrongEKF, ...
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
idxEnd = wrong.m.idxFinal;
hEnd = plot(wrong.q_actual(idxEnd,1), wrong.q_actual(idxEnd,2), 'o', ...
    'MarkerFaceColor', style.cEnd, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'MarkerSize', 8);

xlabel('x [m]');
ylabel('y [m]');
title('XY trajectory - exact vs wrong EKF initialization');

lgd = legend([hRef hExactTrue hExactEKF hWrongTrue hWrongEKF ...
              hTrueStart hEKFStart hCircleStart hEnd], ...
    {'Reference', ...
     'Exact true', 'Exact EKF', ...
     'Wrong-EKF true', 'Wrong-EKF estimate', ...
     'True start', 'EKF initial estimate', 'Circle start', 'End'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 3;
styleLegend(lgd);

setXYLimits(exact, wrong);

saveFigure(outDir, "task4_01_xy_map_exact_vs_wrong_ekf");

%% ============================================================
% Figure 2: Pose states
% ============================================================

fig = figure('Name','States', 'Color','w');
fig.Position = [100 100 style.figW style.figH];

thetaRef = unwrap(exact.q_target(:,3));
thetaExactTrue = unwrap(exact.q_actual(:,3));
thetaExactEKF = unwrap(exact.q_est(:,3));
thetaWrongTrue = unwrap(wrong.q_actual(:,3));
thetaWrongEKF = unwrap(wrong.q_est(:,3));

tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

h1 = plot(exact.t, exact.q_target(:,1), 'k--', 'LineWidth', 1.6);
h2 = plot(exact.t, exact.q_actual(:,1), '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
h3 = plot(exact.t, exact.q_est(:,1), '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
h4 = plot(wrong.t, wrong.q_actual(:,1), '-', 'Color', style.cWrongTrue, 'LineWidth', 1.5);
h5 = plot(wrong.t, wrong.q_est(:,1), ':', 'Color', style.cWrongEKF, 'LineWidth', 1.9);

ylabel('x [m]');
title('Longitudinal position');

lgd = legend([h1 h2 h3 h4 h5], ...
    {'Reference','Exact true','Exact EKF','Wrong-EKF true','Wrong-EKF estimate'}, ...
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
plot(wrong.t, wrong.q_actual(:,2), '-', 'Color', style.cWrongTrue, 'LineWidth', 1.5);
plot(wrong.t, wrong.q_est(:,2), ':', 'Color', style.cWrongEKF, 'LineWidth', 1.9);

ylabel('y [m]');
title('Lateral position');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

plot(exact.t, thetaRef, 'k--', 'LineWidth', 1.6);
plot(exact.t, thetaExactTrue, '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
plot(exact.t, thetaExactEKF, '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
plot(wrong.t, thetaWrongTrue, '-', 'Color', style.cWrongTrue, 'LineWidth', 1.5);
plot(wrong.t, thetaWrongEKF, ':', 'Color', style.cWrongEKF, 'LineWidth', 1.9);

ylabel('\theta [rad]');
xlabel('time [s]');
title('Orientation, unwrapped for visualization');

saveFigure(outDir, "task4_02_states_exact_vs_wrong_ekf");

%% ============================================================
% Figure 3: Errors
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

h3 = plot(wrong.t, wrong.eTrackXY, '-', ...
    'Color', style.cWrongTrue, 'LineWidth', 1.7);

h4 = plot(wrong.t, wrong.eLocXY, '--', ...
    'Color', style.cWrongEKF, 'LineWidth', 1.7);

ylabel('position error [m]');
title('Position errors');

lgd = legend([h1 h2 h3 h4], ...
    {'Exact tracking','Exact localization', ...
     'Wrong-EKF tracking','Wrong-EKF localization'}, ...
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

plot(wrong.t, rad2deg(wrong.eTrackTh), '-', ...
    'Color', style.cWrongTrue, 'LineWidth', 1.7);

plot(wrong.t, rad2deg(wrong.eLocTh), '--', ...
    'Color', style.cWrongEKF, 'LineWidth', 1.7);

ylabel('angle error [deg]');
xlabel('time [s]');
title('Orientation errors');

saveFigure(outDir, "task4_03_errors_exact_vs_wrong_ekf");

%% ============================================================
% Figure 4: Inputs
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

h3 = plot(wrong.t, wrong.v, '-', ...
    'Color', style.cWrongTrue, 'LineWidth', 1.6);

h4 = plot(wrong.t, wrong.omega, '--', ...
    'Color', style.cWrongEKF, 'LineWidth', 1.6);

ylabel('control input');
title('Kinematic control inputs');

lgd = legend([h1 h2 h3 h4], ...
    {'Exact v [m/s]','Exact \omega [rad/s]', ...
     'Wrong-EKF v [m/s]','Wrong-EKF \omega [rad/s]'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 4;
styleLegend(lgd);

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);

wExact = makeNx2(exact.omega_wheels);
wWrong = makeNx2(wrong.omega_wheels);

NwE = min(size(wExact,1), numel(exact.t));
NwW = min(size(wWrong,1), numel(wrong.t));

h1 = plot(exact.t(1:NwE), wExact(1:NwE,1), '-', ...
    'Color', style.cExactTrue, 'LineWidth', 1.6);

h2 = plot(exact.t(1:NwE), wExact(1:NwE,2), '--', ...
    'Color', style.cExactEKF, 'LineWidth', 1.6);

h3 = plot(wrong.t(1:NwW), wWrong(1:NwW,1), '-', ...
    'Color', style.cWrongTrue, 'LineWidth', 1.6);

h4 = plot(wrong.t(1:NwW), wWrong(1:NwW,2), '--', ...
    'Color', style.cWrongEKF, 'LineWidth', 1.6);

yline(10, 'k:', 'LineWidth', 1.1, 'HandleVisibility','off');
yline(-10, 'k:', 'LineWidth', 1.1, 'HandleVisibility','off');

ylabel('wheel speed [rad/s]');
xlabel('time [s]');
title('Wheel speed commands');

lgd = legend([h1 h2 h3 h4], ...
    {'Exact \omega_L','Exact \omega_R', ...
     'Wrong-EKF \omega_L','Wrong-EKF \omega_R'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 4;
styleLegend(lgd);

saveFigure(outDir, "task4_04_inputs_exact_vs_wrong_ekf");

%% ============================================================
% Figure 5: LiDAR innovation
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

h2 = plot(wrong.t, wrong.innovation_RMS, '--', ...
    'Color', style.cWrongEKF, 'LineWidth', 1.5);

ylabel('RMS innovation [m]');
title('LiDAR innovation after gating');

lgd = legend([h1 h2], ...
    {'Exact initialization','Wrong EKF initialization'}, ...
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

h2 = plot(wrong.t, wrong.n_valid_lidar, '--', ...
    'Color', style.cWrongEKF, 'LineWidth', 1.5);

ylabel('valid LiDAR rays');
xlabel('time [s]');
title('Number of LiDAR rays used in EKF update');

lgd = legend([h1 h2], ...
    {'Exact initialization','Wrong EKF initialization'}, ...
    'Location','southoutside', ...
    'Orientation','horizontal');
lgd.NumColumns = 2;
styleLegend(lgd);

saveFigure(outDir, "task4_05_lidar_innovation_exact_vs_wrong_ekf");

%% ============================================================
% Reports
% ============================================================

writeTextReport(fullfile(outDir, "task4_exact_vs_wrong_ekf_metrics.txt"), exact, wrong);
writeLatexTable(fullfile(outDir, "task4_exact_vs_wrong_ekf_table.tex"), exact, wrong);

printMetrics(exact);
printMetrics(wrong);

fprintf('\nFigures and reports saved in:\n%s\n\n', outDir);

%% ========================================================================
% FUNCTIONS
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

    style.cExactTrue = [0.0000 0.5000 0.0000];
    style.cExactEKF  = [0.8500 0.3250 0.0980];

    style.cWrongTrue = [0.0000 0.4470 0.7410];
    style.cWrongEKF  = [0.4940 0.1840 0.5560];

    style.cStart = [0.0000 0.6000 0.0000];
    style.cCircleStart = [0.9290 0.6940 0.1250];
    style.cEnd = [0.6000 0.0000 0.6000];

    style.gridColor = [0.78 0.78 0.78];

    style.cubeFace = [0.86 0.86 0.86];
    style.cubeEdge = [0.00 0.00 0.00];

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

    R.q_target = makeNx3(getSigData(getOutSignal(simOut, "q_target")));
    R.q_actual = makeNx3(getSigData(getOutSignal(simOut, "q_actual")));
    R.q_est    = makeNx3(getSigData(getOutSignal(simOut, "q_est")));

    R.t = getTime(simOut, R.q_est);
    R.t = R.t(:);

    R.v = getSigData(getOutSignal(simOut, "v"));
    R.v = R.v(:);

    R.omega = getSigData(getOutSignal(simOut, "omega"));
    R.omega = R.omega(:);

    R.omega_wheels = makeNx2(getSigData(getOutSignal(simOut, "omega_wheels")));

    R.innovation_RMS = getSigData(getOutSignal(simOut, "innovation_RMS"));
    R.innovation_RMS = R.innovation_RMS(:);

    if hasOutSignal(simOut, "n_valid_lidar_sim")
        R.n_valid_lidar = getSigData(getOutSignal(simOut, "n_valid_lidar_sim"));
    else
        R.n_valid_lidar = getSigData(getOutSignal(simOut, "n_valid_lidar"));
    end
    R.n_valid_lidar = R.n_valid_lidar(:);

    N = min([ ...
        numel(R.t), ...
        size(R.q_target,1), ...
        size(R.q_actual,1), ...
        size(R.q_est,1), ...
        numel(R.v), ...
        numel(R.omega), ...
        size(R.omega_wheels,1), ...
        numel(R.innovation_RMS), ...
        numel(R.n_valid_lidar)]);

    R.t = R.t(1:N);
    R.q_target = R.q_target(1:N,:);
    R.q_actual = R.q_actual(1:N,:);
    R.q_est = R.q_est(1:N,:);
    R.v = R.v(1:N);
    R.omega = R.omega(1:N);
    R.omega_wheels = R.omega_wheels(1:N,:);
    R.innovation_RMS = R.innovation_RMS(1:N);
    R.n_valid_lidar = R.n_valid_lidar(1:N);

end

function tf = hasOutSignal(simOut, name)

    try
        simOut.get(name);
        tf = true;
    catch
        try
            simOut.(name);
            tf = true;
        catch
            tf = false;
        end
    end

end

function sig = getOutSignal(simOut, name)

    try
        sig = simOut.get(name);
    catch
        try
            sig = simOut.(name);
        catch
            error("Signal '%s' was not found in Simulink output.", name);
        end
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

    if hasOutSignal(simOut, "tout")
        t = getOutSignal(simOut, "tout");
    elseif hasOutSignal(simOut, "t")
        t = getSigData(getOutSignal(simOut, "t"));
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

    if isempty(idxFinal)
        idx = true(size(R.t));
        idxFinal = numel(R.t);
    end

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

    R.m.mean_innovation = mean(R.innovation_RMS(idx), "omitnan");
    R.m.max_innovation = max(R.innovation_RMS(idx), [], "omitnan");

    R.m.mean_valid_rays = mean(R.n_valid_lidar(idx), "omitnan");
    R.m.min_valid_rays = min(R.n_valid_lidar(idx), [], "omitnan");
    R.m.max_valid_rays = max(R.n_valid_lidar(idx), [], "omitnan");

    R.m.valid_update_percent = 100 * mean(~isnan(R.innovation_RMS(idx)));
    R.m.skipped_update_percent = 100 - R.m.valid_update_percent;

    % Convergence criterion:
    % localization position error below 0.02 m for at least 1 s.
    Ts_est = median(diff(R.t));
    threshold = 0.02;
    holdTime = 1.0;
    Nhold = max(1, round(holdTime/Ts_est));

    below = R.eLocXY <= threshold;
    conv_idx = NaN;

    for k = 1:(numel(below)-Nhold+1)
        if all(below(k:k+Nhold-1))
            conv_idx = k;
            break;
        end
    end

    if isnan(conv_idx)
        R.m.convergence_time = NaN;
    else
        R.m.convergence_time = R.t(conv_idx);
    end

end

function printMetrics(R)

    fprintf('\nMetrics for %s:\n', R.caseName);
    fprintf('  Duration                         = %.2f s\n', R.m.duration);
    fprintf('  Initial localization position    = %.4f m\n', R.m.initial_loc_xy);
    fprintf('  Initial localization heading     = %.2f deg\n', R.m.initial_loc_heading);
    fprintf('  Tracking RMSE position           = %.4f m\n', R.m.track_rmse_xy);
    fprintf('  Tracking max position error      = %.4f m\n', R.m.track_max_xy);
    fprintf('  Localization RMSE position       = %.4f m\n', R.m.loc_rmse_xy);
    fprintf('  Localization max position        = %.4f m\n', R.m.loc_max_xy);
    fprintf('  Localization final position      = %.4f m\n', R.m.loc_final_xy);
    fprintf('  Localization RMSE heading        = %.2f deg\n', R.m.loc_rmse_heading);
    fprintf('  Max localization heading         = %.2f deg\n', R.m.max_loc_heading);
    fprintf('  Convergence time, e_loc<0.02 m   = %.2f s\n', R.m.convergence_time);
    fprintf('  Max wheel speed                  = %.4f rad/s\n', R.m.max_abs_wheel_speed);
    fprintf('  Mean LiDAR innovation            = %.4f m\n', R.m.mean_innovation);
    fprintf('  Max LiDAR innovation             = %.4f m\n', R.m.max_innovation);
    fprintf('  Mean valid LiDAR rays            = %.2f\n', R.m.mean_valid_rays);
    fprintf('  Valid LiDAR update percent       = %.2f %%\n', R.m.valid_update_percent);

end

function writeTextReport(fileName, exact, wrong)

    fid = fopen(fileName, "w");

    writeOneText(fid, exact);
    fprintf(fid, "\n");
    writeOneText(fid, wrong);

    fclose(fid);

end

function writeOneText(fid, R)

    fprintf(fid, "Metrics for %s:\n", R.caseName);
    fprintf(fid, "Trajectory duration [s] = %.2f\n", R.m.duration);
    fprintf(fid, "Initial tracking position error [m] = %.4f\n", R.m.initial_track_xy);
    fprintf(fid, "Initial tracking heading error [deg] = %.2f\n", R.m.initial_track_heading);
    fprintf(fid, "Initial localization position error [m] = %.4f\n", R.m.initial_loc_xy);
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
    fprintf(fid, "Convergence time below 0.02 m for 1 s [s] = %.2f\n", R.m.convergence_time);
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

function writeLatexTable(fileName, exact, wrong)

    fid = fopen(fileName, "w");

    fprintf(fid, "\\begin{table}[H]\n");
    fprintf(fid, "\\centering\n");
    fprintf(fid, "\\caption{Numerical summary of Task 4 simulation results with wrong EKF initialization.}\n");
    fprintf(fid, "\\label{tab:task4_wrong_ekf_results}\n");
    fprintf(fid, "\\begin{tabular}{lcc}\n");
    fprintf(fid, "\\hline\n");
    fprintf(fid, "Metric & Exact initialization & Wrong EKF initialization \\\\\n");
    fprintf(fid, "\\hline\n");
    fprintf(fid, "Initial localization error [m] & %.4f & %.4f \\\\\n", exact.m.initial_loc_xy, wrong.m.initial_loc_xy);
    fprintf(fid, "Tracking RMSE position [m] & %.4f & %.4f \\\\\n", exact.m.track_rmse_xy, wrong.m.track_rmse_xy);
    fprintf(fid, "Localization RMSE position [m] & %.4f & %.4f \\\\\n", exact.m.loc_rmse_xy, wrong.m.loc_rmse_xy);
    fprintf(fid, "Maximum localization error [m] & %.4f & %.4f \\\\\n", exact.m.loc_max_xy, wrong.m.loc_max_xy);
    fprintf(fid, "Final localization error [m] & %.4f & %.4f \\\\\n", exact.m.loc_final_xy, wrong.m.loc_final_xy);
    fprintf(fid, "Convergence time [s] & %.2f & %.2f \\\\\n", exact.m.convergence_time, wrong.m.convergence_time);
    fprintf(fid, "Maximum LiDAR innovation [m] & %.4f & %.4f \\\\\n", exact.m.max_innovation, wrong.m.max_innovation);
    fprintf(fid, "Mean valid LiDAR rays & %.2f & %.2f \\\\\n", exact.m.mean_valid_rays, wrong.m.mean_valid_rays);
    fprintf(fid, "Maximum wheel speed [rad/s] & %.4f & %.4f \\\\\n", exact.m.max_abs_wheel_speed, wrong.m.max_abs_wheel_speed);
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

function setXYLimits(exact, wrong)

    allX = [exact.q_target(:,1); exact.q_actual(:,1); exact.q_est(:,1); ...
            wrong.q_actual(:,1); wrong.q_est(:,1); 1.80; 2.20];

    allY = [exact.q_target(:,2); exact.q_actual(:,2); exact.q_est(:,2); ...
            wrong.q_actual(:,2); wrong.q_est(:,2); -0.20; 0.20];

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
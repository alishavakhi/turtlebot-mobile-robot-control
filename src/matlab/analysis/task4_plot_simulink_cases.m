%% plot_task4_simulink_exact_vs_nonexact_report_style.m
% Report-style plots for Task 4 Simulink results
% Exact initial point vs non-exact initial point
%
% Output:
% 1) XY trajectory
% 2) states
% 3) errors
% 4) inputs
% 5) LiDAR innovation
%
% Style is kept consistent with previous task plots.

clear; clc; close all;

%% =========================
%  Load .mat files
% ==========================

exactFile = "out.simulink_exact.mat";
nonExactFile = "out.simulink_nonexact.mat";

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

%% =========================
%  Output folder
% ==========================

outDir = "task4_simulink_report_style_plots";

if ~exist(outDir, "dir")
    mkdir(outDir);
end

%% =========================
%  Plot style
% ==========================

setDefaultPlotStyle();

style = getPlotStyle();

% Time limit for all time-domain plots
plotTimeLim = [0 40];

%% =========================
%  Compute errors
% ==========================

exact = computeErrors(exact);
nonExact = computeErrors(nonExact);

%% =========================
%  Figure 1: XY trajectory
% ==========================

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

% Final point
hEnd = plot(nonExact.q_actual(end,1), nonExact.q_actual(end,2), 'o', ...
    'MarkerFaceColor', style.cEnd, ...
    'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, ...
    'MarkerSize', 8);

xlabel('x [m]');
ylabel('y [m]');
title('XY trajectory - exact vs non-exact initial point');

lgd = legend([hRef hExactTrue hExactEKF hNonTrue hNonEKF hStartExact hStartNonExact hEnd], ...
    {'Reference', ...
     'Exact true', 'Exact EKF', ...
     'Non-exact true', 'Non-exact EKF', ...
     'Exact start', 'Non-exact start', 'End'}, ...
    'Location','bestoutside');
styleLegend(lgd);

setXYLimits(exact, nonExact);

saveFigure(outDir, "task4_simulink_01_xy_map_exact_vs_non_exact");

%% =========================
%  Figure 2: States
% ==========================

fig = figure('Name','States', 'Color','w');
fig.Position = [100 100 style.figW style.figH];

t = exact.t;

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
h1 = plot(t, exact.q_target(:,1), 'k--', 'LineWidth', 1.6);
h2 = plot(t, exact.q_actual(:,1), '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
h3 = plot(t, exact.q_est(:,1), '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
h4 = plot(t, nonExact.q_actual(:,1), '-', 'Color', style.cNonTrue, 'LineWidth', 1.5);
h5 = plot(t, nonExact.q_est(:,1), ':', 'Color', style.cNonEKF, 'LineWidth', 1.9);
ylabel('x [m]');
title('Longitudinal position');
lgd = legend([h1 h2 h3 h4 h5], ...
    {'Reference','Exact true','Exact EKF','Non-exact true','Non-exact EKF'}, ...
    'Location','bestoutside');
styleLegend(lgd);

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);
plot(t, exact.q_target(:,2), 'k--', 'LineWidth', 1.6);
plot(t, exact.q_actual(:,2), '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
plot(t, exact.q_est(:,2), '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
plot(t, nonExact.q_actual(:,2), '-', 'Color', style.cNonTrue, 'LineWidth', 1.5);
plot(t, nonExact.q_est(:,2), ':', 'Color', style.cNonEKF, 'LineWidth', 1.9);
ylabel('y [m]');
title('Lateral position');

nexttile;
hold on; grid on;
styleAxes(gca, style);
xlim(plotTimeLim);
plot(t, thetaRef, 'k--', 'LineWidth', 1.6);
plot(t, thetaExactTrue, '-.', 'Color', style.cExactTrue, 'LineWidth', 1.5);
plot(t, thetaExactEKF, '--', 'Color', style.cExactEKF, 'LineWidth', 1.5);
plot(t, thetaNonTrue, '-', 'Color', style.cNonTrue, 'LineWidth', 1.5);
plot(t, thetaNonEKF, ':', 'Color', style.cNonEKF, 'LineWidth', 1.9);
ylabel('\theta [rad]');
xlabel('time [s]');
title('Orientation, unwrapped for visualization');

saveFigure(outDir, "task4_simulink_02_states_exact_vs_non_exact");

%% =========================
%  Figure 3: Errors
% ==========================

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
    'Location','bestoutside');
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

%% =========================
%  Figure 4: Inputs
% ==========================

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
    'Location','bestoutside');
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
    'Location','bestoutside');
styleLegend(lgd);

saveFigure(outDir, "task4_simulink_04_inputs_exact_vs_non_exact");

%% =========================
%  Figure 5: LiDAR innovation
% ==========================

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
    'Location','best');
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
    'Location','best');
styleLegend(lgd);

saveFigure(outDir, "task4_simulink_05_lidar_innovation_exact_vs_non_exact");

%% =========================
%  Numerical report
% ==========================

printMetrics(exact);
printMetrics(nonExact);

writeNumericalReport(outDir, exact, nonExact);

fprintf('\nFigures saved in:\n%s\n\n', outDir);

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
    style.cEnd   = [0.6000 0.0000 0.6000];

    style.gridColor = [0.78 0.78 0.78];

    style.cubeFace = [0.86 0.86 0.86];
    style.cubeEdge = [0.00 0.00 0.00];

    style.figW = 1150;
    style.figH = 820;

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
    lgd.FontSize = 11;

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

    R.v = getSigData(simOut.v);
    R.omega = getSigData(simOut.omega);
    R.omega_wheels = getSigData(simOut.omega_wheels);

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

    R.t = R.t(:);
    R.v = R.v(:);
    R.omega = R.omega(:);

    if ~isempty(R.innovation_RMS)
        R.innovation_RMS = R.innovation_RMS(:);
    end

    if ~isempty(R.n_valid_lidar)
        R.n_valid_lidar = R.n_valid_lidar(:);
    end

    N = min([numel(R.t), size(R.q_target,1), size(R.q_actual,1), size(R.q_est,1)]);

    R.t = R.t(1:N);
    R.q_target = R.q_target(1:N,:);
    R.q_actual = R.q_actual(1:N,:);
    R.q_est = R.q_est(1:N,:);

    if numel(R.v) >= N
        R.v = R.v(1:N);
    end

    if numel(R.omega) >= N
        R.omega = R.omega(1:N);
    end

    if size(R.omega_wheels,1) >= N
        R.omega_wheels = R.omega_wheels(1:N,:);
    end

    if ~isempty(R.innovation_RMS) && numel(R.innovation_RMS) >= N
        R.innovation_RMS = R.innovation_RMS(1:N);
    end

    if ~isempty(R.n_valid_lidar) && numel(R.n_valid_lidar) >= N
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

function S = computeErrors(S)

    S.eTrackXY = sqrt(sum((S.q_actual(:,1:2) - S.q_target(:,1:2)).^2, 2));
    S.eLocXY   = sqrt(sum((S.q_est(:,1:2) - S.q_actual(:,1:2)).^2, 2));

    S.eTrackTh = wrapPiLocal(S.q_target(:,3) - S.q_actual(:,3));
    S.eLocTh   = wrapPiLocal(S.q_est(:,3) - S.q_actual(:,3));

end

function printMetrics(S)

    fprintf('\nMetrics for %s:\n', S.caseName);
    fprintf('  Tracking RMSE position      = %.4f m\n', sqrt(mean(S.eTrackXY.^2)));
    fprintf('  Tracking max position error = %.4f m\n', max(S.eTrackXY));
    fprintf('  Tracking RMSE heading       = %.2f deg\n', rad2deg(sqrt(mean(S.eTrackTh.^2))));
    fprintf('  Localization RMSE position  = %.4f m\n', sqrt(mean(S.eLocXY.^2)));
    fprintf('  Localization max position   = %.4f m\n', max(S.eLocXY));
    fprintf('  Localization RMSE heading   = %.2f deg\n', rad2deg(sqrt(mean(S.eLocTh.^2))));
    fprintf('  Final tracking position     = %.4f m\n', S.eTrackXY(end));
    fprintf('  Final localization position = %.4f m\n', S.eLocXY(end));

    if ~isempty(S.innovation_RMS)
        fprintf('  Mean LiDAR innovation       = %.4f m\n', mean(S.innovation_RMS, 'omitnan'));
        fprintf('  Max LiDAR innovation        = %.4f m\n', max(S.innovation_RMS, [], 'omitnan'));
    end

    if ~isempty(S.n_valid_lidar)
        fprintf('  Mean valid LiDAR rays       = %.2f\n', mean(S.n_valid_lidar, 'omitnan'));
    end

end

function writeNumericalReport(outDir, exact, nonExact)

    reportFile = fullfile(outDir, "task4_simulink_numerical_report.txt");
    fid = fopen(reportFile, "w");

    writeOne(fid, exact);
    fprintf(fid, "\n");
    writeOne(fid, nonExact);

    fclose(fid);

end

function writeOne(fid, S)

    fprintf(fid, "Metrics for %s:\n", S.caseName);
    fprintf(fid, "Tracking RMSE position      = %.4f m\n", sqrt(mean(S.eTrackXY.^2)));
    fprintf(fid, "Tracking max position error = %.4f m\n", max(S.eTrackXY));
    fprintf(fid, "Tracking RMSE heading       = %.2f deg\n", rad2deg(sqrt(mean(S.eTrackTh.^2))));
    fprintf(fid, "Localization RMSE position  = %.4f m\n", sqrt(mean(S.eLocXY.^2)));
    fprintf(fid, "Localization max position   = %.4f m\n", max(S.eLocXY));
    fprintf(fid, "Localization RMSE heading   = %.2f deg\n", rad2deg(sqrt(mean(S.eLocTh.^2))));
    fprintf(fid, "Final tracking position     = %.4f m\n", S.eTrackXY(end));
    fprintf(fid, "Final localization position = %.4f m\n", S.eLocXY(end));

    if ~isempty(S.innovation_RMS)
        fprintf(fid, "Mean LiDAR innovation       = %.4f m\n", mean(S.innovation_RMS, 'omitnan'));
        fprintf(fid, "Max LiDAR innovation        = %.4f m\n", max(S.innovation_RMS, [], 'omitnan'));
    end

    if ~isempty(S.n_valid_lidar)
        fprintf(fid, "Mean valid LiDAR rays       = %.2f\n", mean(S.n_valid_lidar, 'omitnan'));
    end

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



%% Add this after printMetrics(exact); printMetrics(nonExact);

writeTask4LatexTable(outDir, exact, nonExact);

%% Add this function near the end of the plotting script

function writeTask4LatexTable(outDir, exact, nonExact)

    reportFile = fullfile(outDir, "task4_results_table_latex.txt");
    fid = fopen(reportFile, "w");

    exactW = makeNx2(exact.omega_wheels);
    nonW   = makeNx2(nonExact.omega_wheels);

    exactMaxW = max(abs(exactW(:)));
    nonMaxW   = max(abs(nonW(:)));

    exactDuration = max(exact.t);
    nonDuration   = max(nonExact.t);

    exactMeanInnov = mean(exact.innovation_RMS, 'omitnan');
    nonMeanInnov   = mean(nonExact.innovation_RMS, 'omitnan');

    exactMeanValid = mean(exact.n_valid_lidar, 'omitnan');
    nonMeanValid   = mean(nonExact.n_valid_lidar, 'omitnan');

    fprintf(fid, "\\begin{table}[H]\n");
    fprintf(fid, "\\centering\n");
    fprintf(fid, "\\caption{Numerical summary of Task 4 simulation results.}\n");
    fprintf(fid, "\\label{tab:task4_results}\n");
    fprintf(fid, "\\begin{tabular}{lll}\n");
    fprintf(fid, "\\hline\n");
    fprintf(fid, "Metric & Exact initial point & Non-exact initial point \\\\\n");
    fprintf(fid, "\\hline\n");

    fprintf(fid, "Trajectory duration [s] & %.2f & %.2f \\\\\n", exactDuration, nonDuration);
    fprintf(fid, "Tracking RMSE position [m] & %.4f & %.4f \\\\\n", ...
        sqrt(mean(exact.eTrackXY.^2)), sqrt(mean(nonExact.eTrackXY.^2)));
    fprintf(fid, "Tracking max position error [m] & %.4f & %.4f \\\\\n", ...
        max(exact.eTrackXY), max(nonExact.eTrackXY));
    fprintf(fid, "Localization RMSE position [m] & %.4f & %.4f \\\\\n", ...
        sqrt(mean(exact.eLocXY.^2)), sqrt(mean(nonExact.eLocXY.^2)));
    fprintf(fid, "Maximum localization error [m] & %.4f & %.4f \\\\\n", ...
        max(exact.eLocXY), max(nonExact.eLocXY));
    fprintf(fid, "Final localization error [m] & %.4f & %.4f \\\\\n", ...
        exact.eLocXY(end), nonExact.eLocXY(end));
    fprintf(fid, "Localization RMSE heading [deg] & %.2f & %.2f \\\\\n", ...
        rad2deg(sqrt(mean(exact.eLocTh.^2))), ...
        rad2deg(sqrt(mean(nonExact.eLocTh.^2))));
    fprintf(fid, "Maximum wheel speed [rad/s] & %.4f & %.4f \\\\\n", ...
        exactMaxW, nonMaxW);
    fprintf(fid, "Mean LiDAR innovation [m] & %.4f & %.4f \\\\\n", ...
        exactMeanInnov, nonMeanInnov);
    fprintf(fid, "Mean valid LiDAR beams & %.2f & %.2f \\\\\n", ...
        exactMeanValid, nonMeanValid);

    fprintf(fid, "\\hline\n");
    fprintf(fid, "\\end{tabular}\n");
    fprintf(fid, "\\end{table}\n");

    fclose(fid);

end
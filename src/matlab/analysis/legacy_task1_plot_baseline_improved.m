clear; clc; close all;

%% ============================================================
%  TASK 1 - Final report plots for Overleaf
%  Robust version: manual axes + external legend panel
%  Saves all figures inside:
%  same_folder_as_task1_simulation/plots/
% ============================================================

%% ------------------------------------------------------------
% Find Simulink model folder and create plots folder
% ------------------------------------------------------------
modelName = 'task1_simulation';

if bdIsLoaded(modelName)
    modelFile = get_param(modelName,'FileName');
else
    modelFile = which([modelName '.slx']);
end

if isempty(modelFile)
    warning('Simulink model not found. Using current folder.');
    modelFolder = pwd;
else
    modelFolder = fileparts(modelFile);
end

plotsFolder = fullfile(modelFolder, 'plots');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

fprintf('Model folder:\n%s\n\n', modelFolder);
fprintf('Plots will be saved in:\n%s\n\n', plotsFolder);

%% ------------------------------------------------------------
% Style
% ------------------------------------------------------------
set(groot,'defaultFigureColor','w');
set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesXColor','k');
set(groot,'defaultAxesYColor','k');
set(groot,'defaultTextColor','k');
set(groot,'defaultLegendTextColor','k');

c_real = [0.10 0.35 0.80];   % blue
c_base = [0.85 0.25 0.20];   % red
c_imp  = [0.10 0.60 0.25];   % green
c_vic  = [0.95 0.60 0.10];   % orange
c_imu  = [0.45 0.45 0.45];   % grey

q_goal = [-1.60, -1.10, 0];
wrap = @(a) atan2(sin(a), cos(a));

%% ------------------------------------------------------------
% Load real Task 1 data
% ------------------------------------------------------------
Sreal = loadFromModelFolder(modelFolder, 'test1_task1_separated.mat');
Dreal = Sreal.D_task1;

idx0_real = find(Dreal.t >= 0.30, 1, 'first');
if isempty(idx0_real)
    idx0_real = 1;
end

t_real  = Dreal.t(idx0_real:end) - Dreal.t(idx0_real);
q_real  = Dreal.q_est(idx0_real:end,:);
Yv      = Dreal.Y_viscon(idx0_real:end,:);
qimu    = Dreal.q_est_imu(idx0_real:end,:);
v_real  = Dreal.v(idx0_real:end,:);
om_real = Dreal.omega(idx0_real:end,:);
w_real  = Dreal.omega_wheels(idx0_real:end,:);

valid_vicon = ~(abs(Yv(:,1)) < 1e-9 & ...
                abs(Yv(:,2)) < 1e-9 & ...
                abs(Yv(:,3)) < 1e-9);

t_end_real = t_real(end);

%% ------------------------------------------------------------
% Load baseline simulation
% ------------------------------------------------------------
Sbase = loadFromModelFolder(modelFolder, 'exported_task1_sim_baseline_test1.mat');
Dbase = Sbase.Dsim_task1;

t_base  = Dbase.t - Dbase.t(1);
q_base  = Dbase.q_est;
v_base  = Dbase.v;
om_base = Dbase.omega;
w_base  = Dbase.omega_wheels;

%% ------------------------------------------------------------
% Load improved simulation if available
% ------------------------------------------------------------
improvedFile = fullfile(modelFolder, 'exported_task1_sim_improved_test1.mat');
hasImproved = exist(improvedFile, 'file') == 2;

if hasImproved
    Simp = load(improvedFile);

    if isfield(Simp, 'Dsim_task1_improved')
        Dimp = Simp.Dsim_task1_improved;
    elseif isfield(Simp, 'Dsim_task1')
        Dimp = Simp.Dsim_task1;
    else
        error('Improved file exists, but expected variable was not found.');
    end

    t_imp  = Dimp.t - Dimp.t(1);
    q_imp  = Dimp.q_est;
    v_imp  = Dimp.v;
    om_imp = Dimp.omega;
    w_imp  = Dimp.omega_wheels;
else
    fprintf('No improved simulation file found. Plotting only real + baseline.\n');
end

%% ------------------------------------------------------------
% Errors
% ------------------------------------------------------------
ep_real = sqrt((q_goal(1)-q_real(:,1)).^2 + ...
               (q_goal(2)-q_real(:,2)).^2);

ep_base = sqrt((q_goal(1)-q_base(:,1)).^2 + ...
               (q_goal(2)-q_base(:,2)).^2);

eth_real = abs(wrap(q_goal(3)-q_real(:,3)));
eth_base = abs(wrap(q_goal(3)-q_base(:,3)));

if hasImproved
    ep_imp = sqrt((q_goal(1)-q_imp(:,1)).^2 + ...
                  (q_goal(2)-q_imp(:,2)).^2);

    eth_imp = abs(wrap(q_goal(3)-q_imp(:,3)));
end

%% ------------------------------------------------------------
% Time limit
% ------------------------------------------------------------
idx_base_active = find(abs(v_base) > 1e-3 | abs(om_base) > 1e-3, 1, 'last');

if isempty(idx_base_active)
    t_base_end_active = t_base(end);
else
    t_base_end_active = t_base(idx_base_active);
end

if hasImproved
    idx_imp_active = find(abs(v_imp) > 1e-3 | abs(om_imp) > 1e-3, 1, 'last');

    if isempty(idx_imp_active)
        t_imp_end_active = t_imp(end);
    else
        t_imp_end_active = t_imp(idx_imp_active);
    end

    t_plot_end = ceil(max([t_real(end), t_base_end_active, t_imp_end_active]) + 1);
else
    t_plot_end = ceil(max([t_real(end), t_base_end_active]) + 1);
end

t_plot_end = min(t_plot_end, 21);

fprintf('Using time axis limit: 0 to %.1f s\n', t_plot_end);

%% ------------------------------------------------------------
% Metrics
% ------------------------------------------------------------
fprintf('\nTASK 1 METRICS\n');

fprintf('\nReal test1:\n');
fprintf('  final q          = [%.4f %.4f %.4f]\n', q_real(end,1), q_real(end,2), q_real(end,3));
fprintf('  position error   = %.4f m\n', ep_real(end));
fprintf('  theta error      = %.4f rad = %.2f deg\n', eth_real(end), rad2deg(eth_real(end)));
fprintf('  max wheel speed  = %.3f rad/s\n', max(abs(w_real(:))));

fprintf('\nBaseline simulation:\n');
fprintf('  final q          = [%.4f %.4f %.4f]\n', q_base(end,1), q_base(end,2), q_base(end,3));
fprintf('  position error   = %.4f m\n', ep_base(end));
fprintf('  theta error      = %.4f rad = %.2f deg\n', eth_base(end), rad2deg(eth_base(end)));
fprintf('  max wheel speed  = %.3f rad/s\n', max(abs(w_base(:))));

if hasImproved
    fprintf('\nImproved simulation:\n');
    fprintf('  final q          = [%.4f %.4f %.4f]\n', q_imp(end,1), q_imp(end,2), q_imp(end,3));
    fprintf('  position error   = %.4f m\n', ep_imp(end));
    fprintf('  theta error      = %.4f rad = %.2f deg\n', eth_imp(end), rad2deg(eth_imp(end)));
    fprintf('  max wheel speed  = %.3f rad/s\n', max(abs(w_imp(:))));
end

%% ------------------------------------------------------------
% Save metrics
% ------------------------------------------------------------
metricsPath = fullfile(plotsFolder, 'task1_metrics_summary.txt');
fid = fopen(metricsPath, 'w');

fprintf(fid,'TASK 1 METRICS\n');

fprintf(fid,'\nReal test1:\n');
fprintf(fid,'  final q          = [%.4f %.4f %.4f]\n', q_real(end,1), q_real(end,2), q_real(end,3));
fprintf(fid,'  position error   = %.4f m\n', ep_real(end));
fprintf(fid,'  theta error      = %.4f rad = %.2f deg\n', eth_real(end), rad2deg(eth_real(end)));
fprintf(fid,'  max wheel speed  = %.3f rad/s\n', max(abs(w_real(:))));

fprintf(fid,'\nBaseline simulation:\n');
fprintf(fid,'  final q          = [%.4f %.4f %.4f]\n', q_base(end,1), q_base(end,2), q_base(end,3));
fprintf(fid,'  position error   = %.4f m\n', ep_base(end));
fprintf(fid,'  theta error      = %.4f rad = %.2f deg\n', eth_base(end), rad2deg(eth_base(end)));
fprintf(fid,'  max wheel speed  = %.3f rad/s\n', max(abs(w_base(:))));

if hasImproved
    fprintf(fid,'\nImproved simulation:\n');
    fprintf(fid,'  final q          = [%.4f %.4f %.4f]\n', q_imp(end,1), q_imp(end,2), q_imp(end,3));
    fprintf(fid,'  position error   = %.4f m\n', ep_imp(end));
    fprintf(fid,'  theta error      = %.4f rad = %.2f deg\n', eth_imp(end), rad2deg(eth_imp(end)));
    fprintf(fid,'  max wheel speed  = %.3f rad/s\n', max(abs(w_imp(:))));
end

fclose(fid);

%% ============================================================
% FIGURE 1 - XY trajectory
% ============================================================
fig1 = figure('Color','w','Position',[100 100 1200 650]);

ax1 = axes('Parent',fig1,'Position',[0.09 0.12 0.68 0.78]);
hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');

plot(ax1, q_real(:,1), q_real(:,2), '-', ...
     'Color',c_real, 'LineWidth',2.2);

plot(ax1, q_base(:,1), q_base(:,2), '--', ...
     'Color',c_base, 'LineWidth',2.0);

if hasImproved
    plot(ax1, q_imp(:,1), q_imp(:,2), '-.', ...
         'Color',c_imp, 'LineWidth',2.0);
end

plot(ax1, Yv(valid_vicon,1), Yv(valid_vicon,2), '.', ...
     'Color',c_vic, 'MarkerSize',5);

plot(ax1, q_real(1,1), q_real(1,2), 'ko', ...
     'MarkerSize',8, 'LineWidth',1.8);

plot(ax1, q_goal(1), q_goal(2), 'kx', ...
     'MarkerSize',11, 'LineWidth',2.2);

xlabel(ax1,'x [m]');
ylabel(ax1,'y [m]');
title(ax1,'Task 1 - XY Trajectory Comparison');

xlim(ax1,[-2.65 -1.50]);
ylim(ax1,[-1.20 0.60]);

styleAxis(ax1);
addXYLegend(fig1, [0.80 0.32 0.18 0.34], hasImproved, c_real, c_base, c_imp, c_vic);

saveFigure(fig1, plotsFolder, 'task1_xy_trajectory');

%% ============================================================
% FIGURE 2 - Regulation errors
% ============================================================
fig2 = figure('Color','w','Position',[100 100 1200 620]);

ax21 = axes('Parent',fig2,'Position',[0.09 0.56 0.68 0.34]);
hold(ax21,'on'); grid(ax21,'on'); box(ax21,'on');

plot(ax21,t_real,ep_real,'-', ...
     'Color',c_real,'LineWidth',2.0);

plot(ax21,t_base,ep_base,'--', ...
     'Color',c_base,'LineWidth',1.9);

if hasImproved
    plot(ax21,t_imp,ep_imp,'-.', ...
         'Color',c_imp,'LineWidth',1.9);
end

yline(ax21,0.05,'k:','5 cm threshold', ...
      'LineWidth',1.1, ...
      'HandleVisibility','off', ...
      'LabelHorizontalAlignment','left');

addEndLine(ax21,t_end_real);
ylabel(ax21,'Position error [m]');
title(ax21,'Task 1 - Regulation Errors');
xlim(ax21,[0 t_plot_end]);
styleAxis(ax21);

ax22 = axes('Parent',fig2,'Position',[0.09 0.12 0.68 0.34]);
hold(ax22,'on'); grid(ax22,'on'); box(ax22,'on');

plot(ax22,t_real,eth_real,'-', ...
     'Color',c_real,'LineWidth',2.0);

plot(ax22,t_base,eth_base,'--', ...
     'Color',c_base,'LineWidth',1.9);

if hasImproved
    plot(ax22,t_imp,eth_imp,'-.', ...
         'Color',c_imp,'LineWidth',1.9);
end

yline(ax22,0.05,'k:','0.05 rad threshold', ...
      'LineWidth',1.1, ...
      'HandleVisibility','off', ...
      'LabelHorizontalAlignment','left');

addEndLine(ax22,t_end_real);
xlabel(ax22,'Time [s]');
ylabel(ax22,'Orientation error [rad]');
xlim(ax22,[0 t_plot_end]);
styleAxis(ax22);

addCompareLegend(fig2,[0.80 0.36 0.18 0.24],hasImproved,c_real,c_base,c_imp);

saveFigure(fig2, plotsFolder, 'task1_regulation_errors');

%% ============================================================
% FIGURE 3 - State evolution
% ============================================================
fig3 = figure('Color','w','Position',[100 100 1200 760]);

ax31 = axes('Parent',fig3,'Position',[0.09 0.68 0.68 0.24]);
hold(ax31,'on'); grid(ax31,'on'); box(ax31,'on');

plot(ax31,t_real,q_real(:,1),'-','Color',c_real,'LineWidth',2.0);
plot(ax31,t_base,q_base(:,1),'--','Color',c_base,'LineWidth',1.9);
if hasImproved
    plot(ax31,t_imp,q_imp(:,1),'-.','Color',c_imp,'LineWidth',1.9);
end

yline(ax31,q_goal(1),'k:','Goal','HandleVisibility','off');
addEndLine(ax31,t_end_real);
ylabel(ax31,'x [m]');
title(ax31,'Task 1 - State Evolution');
xlim(ax31,[0 t_plot_end]);
styleAxis(ax31);

ax32 = axes('Parent',fig3,'Position',[0.09 0.38 0.68 0.24]);
hold(ax32,'on'); grid(ax32,'on'); box(ax32,'on');

plot(ax32,t_real,q_real(:,2),'-','Color',c_real,'LineWidth',2.0);
plot(ax32,t_base,q_base(:,2),'--','Color',c_base,'LineWidth',1.9);
if hasImproved
    plot(ax32,t_imp,q_imp(:,2),'-.','Color',c_imp,'LineWidth',1.9);
end

yline(ax32,q_goal(2),'k:','Goal','HandleVisibility','off');
addEndLine(ax32,t_end_real);
ylabel(ax32,'y [m]');
xlim(ax32,[0 t_plot_end]);
styleAxis(ax32);

ax33 = axes('Parent',fig3,'Position',[0.09 0.08 0.68 0.24]);
hold(ax33,'on'); grid(ax33,'on'); box(ax33,'on');

plot(ax33,t_real,q_real(:,3),'-','Color',c_real,'LineWidth',2.0);
plot(ax33,t_base,q_base(:,3),'--','Color',c_base,'LineWidth',1.9);
if hasImproved
    plot(ax33,t_imp,q_imp(:,3),'-.','Color',c_imp,'LineWidth',1.9);
end

yline(ax33,q_goal(3),'k:','Goal','HandleVisibility','off');
addEndLine(ax33,t_end_real);
xlabel(ax33,'Time [s]');
ylabel(ax33,'\theta [rad]');
xlim(ax33,[0 t_plot_end]);
styleAxis(ax33);

addCompareLegend(fig3,[0.80 0.39 0.18 0.22],hasImproved,c_real,c_base,c_imp);

saveFigure(fig3, plotsFolder, 'task1_state_evolution');

%% ============================================================
% FIGURE 4 - Control inputs
% ============================================================
fig4 = figure('Color','w','Position',[100 100 1200 620]);

ax41 = axes('Parent',fig4,'Position',[0.09 0.56 0.68 0.34]);
hold(ax41,'on'); grid(ax41,'on'); box(ax41,'on');

plot(ax41,t_real,v_real,'-','Color',c_real,'LineWidth',2.0);
plot(ax41,t_base,v_base,'--','Color',c_base,'LineWidth',1.9);
if hasImproved
    plot(ax41,t_imp,v_imp,'-.','Color',c_imp,'LineWidth',1.9);
end

addEndLine(ax41,t_end_real);
ylabel(ax41,'v [m/s]');
title(ax41,'Task 1 - Control Inputs');
xlim(ax41,[0 t_plot_end]);
styleAxis(ax41);

ax42 = axes('Parent',fig4,'Position',[0.09 0.12 0.68 0.34]);
hold(ax42,'on'); grid(ax42,'on'); box(ax42,'on');

plot(ax42,t_real,om_real,'-','Color',c_real,'LineWidth',2.0);
plot(ax42,t_base,om_base,'--','Color',c_base,'LineWidth',1.9);
if hasImproved
    plot(ax42,t_imp,om_imp,'-.','Color',c_imp,'LineWidth',1.9);
end

addEndLine(ax42,t_end_real);
xlabel(ax42,'Time [s]');
ylabel(ax42,'\omega [rad/s]');
xlim(ax42,[0 t_plot_end]);
styleAxis(ax42);

addCompareLegend(fig4,[0.80 0.36 0.18 0.24],hasImproved,c_real,c_base,c_imp);

saveFigure(fig4, plotsFolder, 'task1_control_inputs');

%% ============================================================
% FIGURE 5 - Wheel speeds
% ============================================================
fig5 = figure('Color','w','Position',[100 100 1200 620]);

ax51 = axes('Parent',fig5,'Position',[0.09 0.56 0.68 0.34]);
hold(ax51,'on'); grid(ax51,'on'); box(ax51,'on');

plot(ax51,t_real,w_real(:,1),'-','Color',c_real,'LineWidth',2.0);
plot(ax51,t_base,w_base(:,1),'--','Color',c_base,'LineWidth',1.9);
if hasImproved
    plot(ax51,t_imp,w_imp(:,1),'-.','Color',c_imp,'LineWidth',1.9);
end

yline(ax51,10,'k:','HandleVisibility','off');
yline(ax51,-10,'k:','HandleVisibility','off');
addEndLine(ax51,t_end_real);
ylabel(ax51,'\omega_L [rad/s]');
title(ax51,'Task 1 - Wheel Speeds');
xlim(ax51,[0 t_plot_end]);
ylim(ax51,[-10.5 10.5]);
styleAxis(ax51);

ax52 = axes('Parent',fig5,'Position',[0.09 0.12 0.68 0.34]);
hold(ax52,'on'); grid(ax52,'on'); box(ax52,'on');

plot(ax52,t_real,w_real(:,2),'-','Color',c_real,'LineWidth',2.0);
plot(ax52,t_base,w_base(:,2),'--','Color',c_base,'LineWidth',1.9);
if hasImproved
    plot(ax52,t_imp,w_imp(:,2),'-.','Color',c_imp,'LineWidth',1.9);
end

yline(ax52,10,'k:','HandleVisibility','off');
yline(ax52,-10,'k:','HandleVisibility','off');
addEndLine(ax52,t_end_real);
xlabel(ax52,'Time [s]');
ylabel(ax52,'\omega_R [rad/s]');
xlim(ax52,[0 t_plot_end]);
ylim(ax52,[-10.5 10.5]);
styleAxis(ax52);

addCompareLegend(fig5,[0.80 0.36 0.18 0.24],hasImproved,c_real,c_base,c_imp);

saveFigure(fig5, plotsFolder, 'task1_wheel_speeds');

%% ============================================================
% FIGURE 6 - Pose estimation diagnostic
% ============================================================
fig6 = figure('Color','w','Position',[100 100 1200 760]);

ax61 = axes('Parent',fig6,'Position',[0.09 0.68 0.68 0.24]);
hold(ax61,'on'); grid(ax61,'on'); box(ax61,'on');

plot(ax61,t_real,q_real(:,1),'-','Color',c_real,'LineWidth',2.0);
plot(ax61,t_real,qimu(:,1),'--','Color',c_imu,'LineWidth',1.6);
plot(ax61,t_real(valid_vicon),Yv(valid_vicon,1),'.','Color',c_vic,'MarkerSize',5);

ylabel(ax61,'x [m]');
title(ax61,'Task 1 - Pose Estimation Diagnostic');
xlim(ax61,[0 t_plot_end]);
styleAxis(ax61);

ax62 = axes('Parent',fig6,'Position',[0.09 0.38 0.68 0.24]);
hold(ax62,'on'); grid(ax62,'on'); box(ax62,'on');

plot(ax62,t_real,q_real(:,2),'-','Color',c_real,'LineWidth',2.0);
plot(ax62,t_real,qimu(:,2),'--','Color',c_imu,'LineWidth',1.6);
plot(ax62,t_real(valid_vicon),Yv(valid_vicon,2),'.','Color',c_vic,'MarkerSize',5);

ylabel(ax62,'y [m]');
xlim(ax62,[0 t_plot_end]);
styleAxis(ax62);

ax63 = axes('Parent',fig6,'Position',[0.09 0.08 0.68 0.24]);
hold(ax63,'on'); grid(ax63,'on'); box(ax63,'on');

plot(ax63,t_real,q_real(:,3),'-','Color',c_real,'LineWidth',2.0);
plot(ax63,t_real,qimu(:,3),'--','Color',c_imu,'LineWidth',1.6);
plot(ax63,t_real(valid_vicon),Yv(valid_vicon,3),'.','Color',c_vic,'MarkerSize',5);

xlabel(ax63,'Time [s]');
ylabel(ax63,'\theta [rad]');
xlim(ax63,[0 t_plot_end]);
styleAxis(ax63);

addPoseLegend(fig6,[0.80 0.39 0.18 0.22],c_real,c_imu,c_vic);

saveFigure(fig6, plotsFolder, 'task1_pose_estimation_diagnostic');

fprintf('\nDONE. Saved all Task 1 figures and metrics inside:\n%s\n', plotsFolder);

%% ============================================================
% Local functions
% ============================================================
function S = loadFromModelFolder(modelFolder, fileName)
    path1 = fullfile(modelFolder, fileName);

    if exist(path1, 'file') == 2
        S = load(path1);
    elseif exist(fileName, 'file') == 2
        S = load(fileName);
    else
        error('Could not find file: %s', fileName);
    end
end

function styleAxis(ax)
    ax.GridAlpha = 0.10;
    ax.GridColor = [0.60 0.60 0.60];
    ax.LineWidth = 0.8;
    ax.FontSize = 10;
    ax.Color = 'w';
end

function addEndLine(ax,t_end_real)
    xline(ax,t_end_real,':', ...
          'Color',[0.45 0.45 0.45], ...
          'LineWidth',0.9, ...
          'HandleVisibility','off');
end

function addCompareLegend(fig,pos,hasImproved,c_real,c_base,c_imp)
    axL = axes('Parent',fig,'Position',pos);
    hold(axL,'on');
    axis(axL,'off');

    h1 = plot(axL,nan,nan,'-','Color',c_real,'LineWidth',2.0);
    h2 = plot(axL,nan,nan,'--','Color',c_base,'LineWidth',1.9);

    if hasImproved
        h3 = plot(axL,nan,nan,'-.','Color',c_imp,'LineWidth',1.9);
        lgd = legend(axL,[h1 h2 h3], ...
                     {'Real robot','Baseline simulation','Improved simulation'}, ...
                     'Location','northwest');
    else
        lgd = legend(axL,[h1 h2], ...
                     {'Real robot','Baseline simulation'}, ...
                     'Location','northwest');
    end

    styleLegend(lgd);
end

function addXYLegend(fig,pos,hasImproved,c_real,c_base,c_imp,c_vic)
    axL = axes('Parent',fig,'Position',pos);
    hold(axL,'on');
    axis(axL,'off');

    h1 = plot(axL,nan,nan,'-','Color',c_real,'LineWidth',2.2);
    h2 = plot(axL,nan,nan,'--','Color',c_base,'LineWidth',2.0);

    if hasImproved
        h3 = plot(axL,nan,nan,'-.','Color',c_imp,'LineWidth',2.0);
    end

    h4 = plot(axL,nan,nan,'.','Color',c_vic,'MarkerSize',8);
    h5 = plot(axL,nan,nan,'ko','MarkerSize',8,'LineWidth',1.8,'LineStyle','none');
    h6 = plot(axL,nan,nan,'kx','MarkerSize',11,'LineWidth',2.2,'LineStyle','none');

    if hasImproved
        lgd = legend(axL,[h1 h2 h3 h4 h5 h6], ...
                     {'Real robot','Baseline simulation','Improved simulation', ...
                      'Vicon measurements','Start','Goal'}, ...
                     'Location','northwest');
    else
        lgd = legend(axL,[h1 h2 h4 h5 h6], ...
                     {'Real robot','Baseline simulation', ...
                      'Vicon measurements','Start','Goal'}, ...
                     'Location','northwest');
    end

    styleLegend(lgd);
end

function addPoseLegend(fig,pos,c_real,c_imu,c_vic)
    axL = axes('Parent',fig,'Position',pos);
    hold(axL,'on');
    axis(axL,'off');

    h1 = plot(axL,nan,nan,'-','Color',c_real,'LineWidth',2.0);
    h2 = plot(axL,nan,nan,'--','Color',c_imu,'LineWidth',1.6);
    h3 = plot(axL,nan,nan,'.','Color',c_vic,'MarkerSize',8);

    lgd = legend(axL,[h1 h2 h3], ...
                 {'Estimated pose q_{est}', ...
                  'Odometry/IMU estimate', ...
                  'Vicon measurements'}, ...
                 'Location','northwest');

    styleLegend(lgd);
end

function styleLegend(lgd)
    lgd.Color = 'w';
    lgd.TextColor = 'k';
    lgd.EdgeColor = [0.7 0.7 0.7];
    lgd.FontSize = 9;
end

function saveFigure(fig, plotsFolder, baseName)
    drawnow;

    pngPath = fullfile(plotsFolder, [baseName '.png']);
    pdfPath = fullfile(plotsFolder, [baseName '.pdf']);

    try
        exportgraphics(fig, pngPath, 'Resolution',300);
        fprintf('Saved PNG: %s\n', pngPath);
    catch ME
        warning('Could not save PNG for %s: %s', baseName, ME.message);
    end

    try
        exportgraphics(fig, pdfPath, 'ContentType','vector');
        fprintf('Saved PDF: %s\n', pdfPath);
    catch ME
        warning('Could not save PDF for %s: %s', baseName, ME.message);
    end
end
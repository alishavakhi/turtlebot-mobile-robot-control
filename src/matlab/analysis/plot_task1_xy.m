clear; clc; close all;

% ===============================
% Report style: white background
% ===============================
set(groot,'defaultFigureColor','w');
set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesXColor','k');
set(groot,'defaultAxesYColor','k');
set(groot,'defaultTextColor','k');
set(groot,'defaultLegendTextColor','k');

load('test2_tasks_separated.mat');

D = D_task1;

% Remove initial localization synchronization transient
idx0 = find(D.t >= 0.30, 1, 'first');

t        = D.t(idx0:end);
q_est    = D.q_est(idx0:end,:);
Yv       = D.Y_viscon(idx0:end,:);

% Goal of Task 1
q_goal = [-1.60, -1.10, 0];

% Remove missing Vicon measurements [0 0 0]
valid_vicon = ~(abs(Yv(:,1)) < 1e-9 & ...
                abs(Yv(:,2)) < 1e-9 & ...
                abs(Yv(:,3)) < 1e-9);

% ===============================
% Plot
% ===============================
fig = figure('Color','w','Position',[100 100 850 600]);
hold on; grid on; box on;

% Initial area of Task 1
% x1 <= x <= x3, y6 <= y <= y8
x1 = -2.90; x3 = -1.50;
y6 =  0.10; y8 =  1.50;

rectangle('Position',[x1 y6 x3-x1 y8-y6], ...
          'FaceColor',[0 0.8 0 0.08], ...
          'EdgeColor',[0 0.5 0], ...
          'LineWidth',1.2);

% Estimated trajectory
h1 = plot(q_est(:,1), q_est(:,2), 'b-', 'LineWidth', 2.2);

% Vicon measurements
h2 = plot(Yv(valid_vicon,1), Yv(valid_vicon,2), '.', ...
          'Color',[0.95 0.55 0], ...
          'MarkerSize',8);

% Start, final, goal
h3 = plot(q_est(1,1), q_est(1,2), 'go', ...
          'MarkerSize',9, 'LineWidth',2);

h4 = plot(q_est(end,1), q_est(end,2), 'ro', ...
          'MarkerSize',9, 'LineWidth',2);

h5 = plot(q_goal(1), q_goal(2), 'kx', ...
          'MarkerSize',12, 'LineWidth',2.5);

% Draw a light line from start to goal only as visual reference
h6 = plot([q_est(1,1) q_goal(1)], [q_est(1,2) q_goal(2)], ...
          'k--', 'LineWidth',1.2);

xlabel('x [m]','FontSize',12);
ylabel('y [m]','FontSize',12);
title('Task 1 - Real Robot Navigation to Configuration 1','FontSize',13);

% Better view for vertical trajectory
xlim([-2.05 -1.35]);
ylim([-1.20 1.15]);

axis equal;

legend([h6 h1 h2 h3 h4 h5], ...
       {'Reference direction', ...
        'Estimated trajectory', ...
        'Vicon measurements', ...
        'Start', ...
        'Final', ...
        'Goal'}, ...
       'Location','eastoutside', ...
       'Color','w', ...
       'TextColor','k', ...
       'EdgeColor','k');

set(gca,'FontSize',11);
set(gca,'LineWidth',1);
set(gca,'Color','w');

% Save high-resolution figure for report
exportgraphics(fig,'task1_xy_real_white.png','Resolution',300);

disp('Saved: task1_xy_real_white.png');
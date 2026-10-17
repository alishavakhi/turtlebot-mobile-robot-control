clear; clc; close all;

%% ============================================================
% Check Vicon data loss in full real robot test
% and compare loss inside Task 2 vs after Task 2.
%
% Input:
%   realrobot_test3.mat
%
% This script is robust for:
%   - Simulink.SimulationOutput variable named "out"
%   - timeseries signals
%   - numeric exported signals
% ============================================================

fullFile = 'realrobot_test3.mat';

if ~isfile(fullFile)
    error('File not found: %s', fullFile);
end

S = load(fullFile);

fprintf('\nLoaded file: %s\n', fullFile);
fprintf('Variables inside file:\n');
disp(fieldnames(S));

%% --------- GET DATA SOURCE ---------
if isfield(S, 'out')
    src = S.out;
    fprintf('\nUsing source: S.out\n');

    if isa(src, 'Simulink.SimulationOutput')
        fprintf('Detected Simulink.SimulationOutput.\n');
        fprintf('Signals inside out:\n');
        disp(src.who.');
    else
        fprintf('S.out exists but is not Simulink.SimulationOutput.\n');
        disp(class(src));
    end
else
    src = S;
    fprintf('\nNo variable named out found. Using loaded structure S directly.\n');
end

%% --------- EXTRACT SIGNALS ---------
q_target_raw = getSignal(src, S, {'q_target'});
Y_vicon_raw  = getSignal(src, S, {'Y_viscon', 'Y_vicon', 'Y_vicon_used', 'Y_vicon_turtlebot'});

q_target = forceCols(q_target_raw, 3);
Y_vicon  = forceCols(Y_vicon_raw, 3);

N = min(size(q_target,1), size(Y_vicon,1));
q_target = q_target(1:N,:);
Y_vicon  = Y_vicon(1:N,:);

% Time
try
    t_raw = getSignal(src, S, {'tout'});
    t = squeeze(t_raw);
    t = t(:);
catch
    try
        t_raw = getSignal(src, S, {'t'});
        t = squeeze(t_raw);
        t = t(:);
    catch
        t = (0:N-1).' * 0.04;
    end
end

t = t(1:min(numel(t),N));
N = min(N, numel(t));
q_target = q_target(1:N,:);
Y_vicon  = Y_vicon(1:N,:);
t = t(1:N);

fprintf('\nFull samples: %d\n', N);
fprintf('Full duration: %.3f s\n', t(end)-t(1));

%% --------- TASK 2 TARGET DEFINITIONS ---------
q_wp1   = [-0.20; -1.10; 0];
q_wp2   = [-0.20; -0.30; pi/2];
q_goal2 = [ 1.00; -0.30; 0];
q_wp3   = [ 1.45; -0.30; 0];

targetTol = 1e-3;

fprintf('\nUnique q_target rows in full test:\n');
disp(unique(round(q_target,3), 'rows'));

%% --------- DETECT TASK 2 START AND END ---------
is_wp1 = vecnorm(q_target(:,1:2) - q_wp1(1:2).', 2, 2) < targetTol;
idxStart = find(is_wp1, 1, 'first');

if isempty(idxStart)
    error('Could not detect Task 2 start: q_wp1 never appears as q_target.');
end

is_wp3 = vecnorm(q_target(:,1:2) - q_wp3(1:2).', 2, 2) < targetTol;
idxWp3 = find(is_wp3 & ((1:N).' > idxStart), 1, 'first');

if ~isempty(idxWp3)
    idxEnd = idxWp3 - 1;
else
    is_goal2 = vecnorm(q_target(:,1:2) - q_goal2(1:2).', 2, 2) < targetTol;
    idxGoal2Last = find(is_goal2 & ((1:N).' > idxStart), 1, 'last');

    if isempty(idxGoal2Last)
        warning('Could not detect q_wp3 or q_goal2 after Task 2 start. Using end of file.');
        idxEnd = N;
    else
        idxEnd = idxGoal2Last;
    end
end

fprintf('\nDetected Task 2 interval:\n');
fprintf('idxStart = %d, tStart = %.3f s\n', idxStart, t(idxStart));
fprintf('idxEnd   = %d, tEnd   = %.3f s\n', idxEnd, t(idxEnd));
fprintf('Task 2 duration = %.3f s\n', t(idxEnd)-t(idxStart));

%% --------- DETECT VICON LOSS ---------
% In this lab setup, missing Vicon is represented by [0;0;0].
lost = all(abs(Y_vicon) < 1e-12, 2);

loss_task2 = lost(idxStart:idxEnd);
loss_before_task2 = lost(1:idxStart-1);
loss_after_task2 = lost(idxEnd+1:end);

fprintf('\n============================================================\n');
fprintf('VICON LOSS CHECK\n');
fprintf('============================================================\n');

fprintf('Total loss samples in full test:    %d / %d = %.2f %%\n', ...
        sum(lost), N, 100*sum(lost)/N);

fprintf('Loss samples before Task 2:         %d / %d = %.2f %%\n', ...
        sum(loss_before_task2), numel(loss_before_task2), ...
        safePercent(sum(loss_before_task2), numel(loss_before_task2)));

fprintf('Loss samples inside Task 2:         %d / %d = %.2f %%\n', ...
        sum(loss_task2), numel(loss_task2), ...
        safePercent(sum(loss_task2), numel(loss_task2)));

fprintf('Loss samples after Task 2:          %d / %d = %.2f %%\n', ...
        sum(loss_after_task2), numel(loss_after_task2), ...
        safePercent(sum(loss_after_task2), numel(loss_after_task2)));

idxLoss = find(lost);

if isempty(idxLoss)
    fprintf('\nNo Vicon loss found in the full test.\n');
else
    fprintf('\nFirst Vicon loss:\n');
    fprintf('idx = %d, t = %.3f s\n', idxLoss(1), t(idxLoss(1)));

    if idxLoss(1) < idxStart
        fprintf('First loss occurred BEFORE Task 2.\n');
    elseif idxLoss(1) <= idxEnd
        fprintf('First loss occurred DURING Task 2.\n');
    else
        fprintf('First loss occurred AFTER Task 2.\n');
    end
end

%% --------- LOSS INTERVALS ---------
lossIntervals = maskIntervals(t, lost);

fprintf('\nLoss intervals in full test [start_time, end_time]:\n');
if isempty(lossIntervals)
    disp('None');
else
    disp(lossIntervals);
end

%% --------- QUICK PLOT ---------
fig = figure('Color','w', 'Name','Vicon availability full test');
hold on; grid on;

stairs(t, double(~lost), 'b', 'LineWidth', 1.5);

xline(t(idxStart), 'g--', 'LineWidth', 1.5);
xline(t(idxEnd),   'r--', 'LineWidth', 1.5);

ylim([-0.1 1.1]);
xlabel('time [s]');
ylabel('Vicon valid');
title('Full real test - Vicon availability');

legend('Vicon valid', 'Task 2 start', 'Task 2 end', 'Location','best');

outDir = fullfile(pwd, 'plots_task2');

if ~exist(outDir,'dir')
    mkdir(outDir);
end

saveas(fig, fullfile(outDir, 'task2_vicon_loss_full_test_check.png'));
savefig(fig, fullfile(outDir, 'task2_vicon_loss_full_test_check.fig'));

fprintf('\nSaved plot:\n%s\n', fullfile(outDir, 'task2_vicon_loss_full_test_check.png'));

%% ============================================================
% Helper functions
% ============================================================

function sig = getSignal(src, S, names)

    sig = [];

    for i = 1:numel(names)
        name = names{i};

        if isa(src, 'Simulink.SimulationOutput')
            available = src.who;

            if any(strcmp(available, name))
                sigObj = src.get(name);
                sig = getRaw(sigObj);
                return;
            end

        elseif isstruct(src)
            if isfield(src, name)
                sig = getRaw(src.(name));
                return;
            end
        end

        if isstruct(S) && isfield(S, name)
            sig = getRaw(S.(name));
            return;
        end
    end

    error('None of these signals were found: %s', strjoin(names, ', '));
end

function X = getRaw(sig)

    if isa(sig, 'timeseries')
        X = squeeze(sig.Data);

    elseif isa(sig, 'Simulink.SimulationData.Signal')
        X = squeeze(sig.Values.Data);

    elseif isnumeric(sig)
        X = squeeze(sig);

    elseif isstruct(sig) && isfield(sig, 'signals')
        X = squeeze(sig.signals.values);

    elseif isstruct(sig) && isfield(sig, 'Data')
        X = squeeze(sig.Data);

    else
        error('Unsupported signal format: %s', class(sig));
    end
end

function X = forceCols(X, ncols)

    X = squeeze(X);

    if isvector(X)
        X = X(:).';
    end

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Expected %d columns, got size %dx%d.', ncols, size(X,1), size(X,2));
    end
end

function p = safePercent(a, b)
    if b == 0
        p = 0;
    else
        p = 100 * a / b;
    end
end

function intervals = maskIntervals(t, mask)

    mask = mask(:);

    if ~any(mask)
        intervals = [];
        return;
    end

    d = diff([false; mask; false]);
    idxStart = find(d == 1);
    idxEnd   = find(d == -1) - 1;

    intervals = [t(idxStart), t(idxEnd)];
end

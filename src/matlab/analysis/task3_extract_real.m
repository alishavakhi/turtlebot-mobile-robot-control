%% extract_task3_real_from_full.m
% Extract the Task 3 real segment from a full real-robot run.
%
% Input:
%   realrobot_test3.mat
%
% Output:
%   real_task3_from_full.mat
%
% The script tries to separate the segment after Task 2 finishes.
% If automatic segmentation is wrong, set manualStartTime_sec or manualStartIndex.

clear; clc; close all;

%% ============================================================
% USER SETTINGS
% ============================================================
realFile = 'realrobot_test3.mat';
outFile  = 'real_task3_from_full.mat';

Ts_default = 0.04;
r_default  = 0.03;
d_default  = 0.165;
omegaM_default = 10;

% Task 2 final goal from the final activity
q_goal_task2 = [1.00; -0.30; 0.00];

% Manual segmentation override.
% Keep empty [] first. If automatic segmentation is wrong, fill one of these.
manualStartIndex    = [];
manualStartTime_sec = [];
manualEndIndex      = [];
manualEndTime_sec   = [];

% Automatic detection thresholds
targetPosTol = 0.08;     % [m]
targetThetaTol = 0.35;   % [rad]
robotPosTol = 0.15;      % [m]

%% ============================================================
% LOAD FILE
% ============================================================
if ~isfile(realFile)
    error('File not found: %s', realFile);
end

L = load(realFile);

fprintf('\nLoaded file: %s\n', realFile);
fprintf('Variables inside file:\n');
disp(fieldnames(L));

source = findSimulationOutput(L);

if isa(source, 'Simulink.SimulationOutput')
    fprintf('\nDetected Simulink.SimulationOutput.\n');
    try
        fprintf('Output names:\n');
        disp(source.who);
    catch
    end
else
    fprintf('\nNo Simulink.SimulationOutput detected. Reading direct variables/structs.\n');
end

%% ============================================================
% EXTRACT SIGNALS
% ============================================================
q_est = getSignal(source, L, {'q_est'});
q_est = toNxM(q_est, 3, 'q_est');

q_target = getSignal(source, L, {'q_target'});
q_target = toNxM(q_target, 3, 'q_target');

Y_gt = getSignal(source, L, { ...
    'Y_viscon_ground_trith', ...
    'Y_viscon_ground_truth', ...
    'Y_vicon_ground_truth', ...
    'Y_vicon_ground_truth_sim', ...
    'Y_vicon_ground_truth_real'});
if isempty(Y_gt)
    warning('Ground truth Vicon not found. Using q_est as fallback.');
    Y_gt = q_est;
else
    Y_gt = toNxM(Y_gt, 3, 'Y_vicon_ground_truth');
end

Y_vicon = getSignal(source, L, { ...
    'Y_viscon', ...
    'Y_vicon_used', ...
    'Y_vicon_turtlebot', ...
    'Y_vicon_turtulbot'});
if isempty(Y_vicon)
    warning('Robot-side Vicon not found. Using ground truth as fallback.');
    Y_vicon = Y_gt;
else
    Y_vicon = toNxM(Y_vicon, 3, 'Y_vicon_turtlebot');
end

q_est_imu = getSignal(source, L, {'q_est_imu'});
if isempty(q_est_imu)
    q_est_imu = NaN(size(q_est));
else
    q_est_imu = toNxM(q_est_imu, 3, 'q_est_imu');
end

v = getSignal(source, L, {'v'});
v = toCol(v, 'v');

omega = getSignal(source, L, {'omega'});
omega = toCol(omega, 'omega');

omega_wheels = getSignal(source, L, {'omega_wheels'});
if isempty(omega_wheels)
    omega_wheels = wheelsFromVOmega(v, omega, r_default, d_default);
else
    omega_wheels = toNxM(omega_wheels, 2, 'omega_wheels');
end

bot_wheelspeed = getSignal(source, L, { ...
    'bot_wheelspeed', ...
    'bot_wheelspeeds', ...
    'bot_wheelspeed_real'});
if isempty(bot_wheelspeed)
    bot_wheelspeed = NaN(size(omega_wheels));
else
    bot_wheelspeed = toNxM(bot_wheelspeed, 2, 'bot_wheelspeed');
end

u_actual = getSignal(source, L, {'u_actual'});
if isempty(u_actual)
    u_actual = [v(:), omega(:)];
else
    u_actual = toNxM(u_actual, 2, 'u_actual');
end

state_out = getSignal(source, L, {'state_out'});
if isempty(state_out)
    state_out = [];
else
    state_out = toCol(state_out, 'state_out');
end

task3_active = getSignal(source, L, {'task3_active'});
if isempty(task3_active)
    task3_active = [];
else
    task3_active = toCol(task3_active, 'task3_active');
end

task3_done = getSignal(source, L, {'task3_done'});
if isempty(task3_done)
    task3_done = [];
else
    task3_done = toCol(task3_done, 'task3_done');
end

elapsed_circle = getSignal(source, L, {'elapsed_circle'});
if isempty(elapsed_circle)
    elapsed_circle = [];
else
    elapsed_circle = toCol(elapsed_circle, 'elapsed_circle');
end

t = getSignal(source, L, {'t', 'tout'});
if isempty(t)
    Nbase = max([size(q_est,1), size(Y_gt,1), size(q_target,1)]);
    t = (0:Nbase-1).' * Ts_default;
else
    t = toCol(t, 't');
end

%% ============================================================
% ALIGN ALL SIGNAL LENGTHS
% ============================================================
N = max([ ...
    size(q_est,1), ...
    size(Y_gt,1), ...
    size(Y_vicon,1), ...
    size(q_target,1), ...
    numel(t)]);

t = fitRows(t, N);
q_est = fitRows(q_est, N);
q_target = fitRows(q_target, N);
Y_gt = fitRows(Y_gt, N);
Y_vicon = fitRows(Y_vicon, N);
q_est_imu = fitRows(q_est_imu, N);
v = fitRows(v, N);
omega = fitRows(omega, N);
omega_wheels = fitRows(omega_wheels, N);
bot_wheelspeed = fitRows(bot_wheelspeed, N);
u_actual = fitRows(u_actual, N);

if ~isempty(state_out), state_out = fitRows(state_out, N); end
if ~isempty(task3_active), task3_active = fitRows(task3_active, N); end
if ~isempty(task3_done), task3_done = fitRows(task3_done, N); end
if ~isempty(elapsed_circle), elapsed_circle = fitRows(elapsed_circle, N); end

t = t - t(1);

q_est(:,3) = wrapToPiLocal(q_est(:,3));
q_target(:,3) = wrapToPiLocal(q_target(:,3));
Y_gt(:,3) = wrapToPiLocal(Y_gt(:,3));
Y_vicon(:,3) = wrapToPiLocal(Y_vicon(:,3));
q_est_imu(:,3) = wrapToPiLocal(q_est_imu(:,3));

fprintf('\nFull extracted run:\n');
fprintf('Samples = %d\n', N);
fprintf('Duration = %.3f s\n', t(end));
fprintf('Initial target = [%.3f %.3f %.3f]\n', q_target(1,1), q_target(1,2), q_target(1,3));
fprintf('Final target   = [%.3f %.3f %.3f]\n', q_target(end,1), q_target(end,2), q_target(end,3));
fprintf('Initial GT     = [%.3f %.3f %.3f]\n', Y_gt(1,1), Y_gt(1,2), Y_gt(1,3));
fprintf('Final GT       = [%.3f %.3f %.3f]\n', Y_gt(end,1), Y_gt(end,2), Y_gt(end,3));

%% ============================================================
% SEGMENTATION: FROM END OF TASK 2 TO END
% ============================================================
if ~isempty(manualStartIndex)
    idxStart = manualStartIndex;
    cropReason = 'manualStartIndex';
elseif ~isempty(manualStartTime_sec)
    [~, idxStart] = min(abs(t - manualStartTime_sec));
    cropReason = 'manualStartTime_sec';
else
    [idxStart, cropReason] = detectTask3Start( ...
        t, q_target, Y_gt, q_goal_task2, ...
        targetPosTol, targetThetaTol, robotPosTol, ...
        task3_active, state_out);
end

if isempty(idxStart) || isnan(idxStart) || idxStart < 1 || idxStart > N
    warning('Automatic segmentation failed. Falling back to idxStart = 1.');
    idxStart = 1;
    cropReason = 'fallback_full_run';
end

if ~isempty(manualEndIndex)
    idxEnd = manualEndIndex;
elseif ~isempty(manualEndTime_sec)
    [~, idxEnd] = min(abs(t - manualEndTime_sec));
else
    idxEnd = N;
end

idxStart = max(1, min(idxStart, N));
idxEnd = max(idxStart, min(idxEnd, N));
idx = idxStart:idxEnd;

fprintf('\nSegmentation result:\n');
fprintf('Reason   = %s\n', cropReason);
fprintf('idxStart = %d, tStart = %.3f s\n', idxStart, t(idxStart));
fprintf('idxEnd   = %d, tEnd   = %.3f s\n', idxEnd, t(idxEnd));
fprintf('Exported samples = %d\n', numel(idx));

%% ============================================================
% BUILD STRUCTURES
% ============================================================
Dreal_full = struct();
Dreal_full.t = t;
Dreal_full.q_est = q_est;
Dreal_full.q_target = q_target;
Dreal_full.Y_vicon_ground_truth = Y_gt;
Dreal_full.Y_vicon_turtlebot = Y_vicon;
Dreal_full.q_est_imu = q_est_imu;
Dreal_full.v = v;
Dreal_full.omega = omega;
Dreal_full.omega_wheels = omega_wheels;
Dreal_full.bot_wheelspeed = bot_wheelspeed;
Dreal_full.u_actual = u_actual;
Dreal_full.state_out = state_out;
Dreal_full.task3_active = task3_active;
Dreal_full.task3_done = task3_done;
Dreal_full.elapsed_circle = elapsed_circle;

Dreal_task3 = struct();
Dreal_task3.t = t(idx) - t(idxStart);

Dreal_task3.q_est = q_est(idx,:);
Dreal_task3.q_target = q_target(idx,:);
Dreal_task3.Y_vicon_ground_truth = Y_gt(idx,:);
Dreal_task3.Y_vicon_turtlebot = Y_vicon(idx,:);
Dreal_task3.Y_viscon_ground_trith = Y_gt(idx,:);
Dreal_task3.Y_viscon = Y_vicon(idx,:);

Dreal_task3.q_est_imu = q_est_imu(idx,:);
Dreal_task3.v = v(idx,:);
Dreal_task3.omega = omega(idx,:);
Dreal_task3.omega_wheels = omega_wheels(idx,:);
Dreal_task3.bot_wheelspeed = bot_wheelspeed(idx,:);
Dreal_task3.u_actual = u_actual(idx,:);

if ~isempty(state_out), Dreal_task3.state_out = state_out(idx,:); end
if ~isempty(task3_active), Dreal_task3.task3_active = task3_active(idx,:); end
if ~isempty(task3_done), Dreal_task3.task3_done = task3_done(idx,:); end
if ~isempty(elapsed_circle), Dreal_task3.elapsed_circle = elapsed_circle(idx,:); end

Dreal_task3.Ts = Ts_default;
Dreal_task3.r = r_default;
Dreal_task3.d = d_default;
Dreal_task3.omega_M = omegaM_default;
Dreal_task3.q_goal_task2 = q_goal_task2;

Dreal_task3.idx_start_original = idxStart;
Dreal_task3.idx_end_original = idxEnd;
Dreal_task3.t_start_original = t(idxStart);
Dreal_task3.t_end_original = t(idxEnd);
Dreal_task3.cropReason = cropReason;
Dreal_task3.source_file = realFile;

%% ============================================================
% REGION FLAGS AND DATA LOSS
% ============================================================
x = Dreal_task3.Y_vicon_ground_truth(:,1);
y = Dreal_task3.Y_vicon_ground_truth(:,2);

Dreal_task3.in_red = double(x >= -0.60 & x <= 0.20 & y >= -1.50 & y <= 0.10);
Dreal_task3.in_orange = double(x >= 1.00 & x <= 2.90 & y >= -1.50 & y <= 1.50);

Dreal_task3.loss_flag = double(all(abs(Dreal_task3.Y_vicon_turtlebot) < 1e-12, 2));
Dreal_task3.vicon_valid = 1 - Dreal_task3.loss_flag;

Dreal_task3.loss_intervals = maskIntervals(Dreal_task3.t, Dreal_task3.loss_flag > 0.5);
Dreal_task3.red_intervals = maskIntervals(Dreal_task3.t, Dreal_task3.in_red > 0.5);
Dreal_task3.orange_intervals = maskIntervals(Dreal_task3.t, Dreal_task3.in_orange > 0.5);

%% ============================================================
% FILTER PERFORMANCE METRICS
% ============================================================
e_target_pos = vecnorm(Dreal_task3.Y_vicon_ground_truth(:,1:2) - Dreal_task3.q_target(:,1:2), 2, 2);
e_target_theta = wrapToPiLocal(Dreal_task3.Y_vicon_ground_truth(:,3) - Dreal_task3.q_target(:,3));

e_ekf_pos = vecnorm(Dreal_task3.q_est(:,1:2) - Dreal_task3.Y_vicon_ground_truth(:,1:2), 2, 2);
e_ekf_theta = wrapToPiLocal(Dreal_task3.q_est(:,3) - Dreal_task3.Y_vicon_ground_truth(:,3));

if all(isnan(Dreal_task3.q_est_imu(:)))
    e_imu_pos = NaN(size(e_ekf_pos));
    e_imu_theta = NaN(size(e_ekf_theta));
else
    e_imu_pos = vecnorm(Dreal_task3.q_est_imu(:,1:2) - Dreal_task3.Y_vicon_ground_truth(:,1:2), 2, 2);
    e_imu_theta = wrapToPiLocal(Dreal_task3.q_est_imu(:,3) - Dreal_task3.Y_vicon_ground_truth(:,3));
end

validVicon = Dreal_task3.vicon_valid > 0.5;
e_raw_vicon_pos = NaN(size(e_ekf_pos));
e_raw_vicon_theta = NaN(size(e_ekf_theta));

e_raw_vicon_pos(validVicon) = vecnorm( ...
    Dreal_task3.Y_vicon_turtlebot(validVicon,1:2) - Dreal_task3.Y_vicon_ground_truth(validVicon,1:2), 2, 2);

e_raw_vicon_theta(validVicon) = wrapToPiLocal( ...
    Dreal_task3.Y_vicon_turtlebot(validVicon,3) - Dreal_task3.Y_vicon_ground_truth(validVicon,3));

Dreal_task3.filter_errors.e_target_pos = e_target_pos;
Dreal_task3.filter_errors.e_target_theta = e_target_theta;
Dreal_task3.filter_errors.e_ekf_pos = e_ekf_pos;
Dreal_task3.filter_errors.e_ekf_theta = e_ekf_theta;
Dreal_task3.filter_errors.e_imu_pos = e_imu_pos;
Dreal_task3.filter_errors.e_imu_theta = e_imu_theta;
Dreal_task3.filter_errors.e_raw_vicon_pos = e_raw_vicon_pos;
Dreal_task3.filter_errors.e_raw_vicon_theta = e_raw_vicon_theta;

Dreal_task3.metrics.duration_s = Dreal_task3.t(end);
Dreal_task3.metrics.target_rmse_m = sqrt(mean(e_target_pos.^2,'omitnan'));
Dreal_task3.metrics.target_max_m = max(e_target_pos,[],'omitnan');
Dreal_task3.metrics.target_final_m = e_target_pos(end);
Dreal_task3.metrics.theta_rmse_rad = sqrt(mean(e_target_theta.^2,'omitnan'));
Dreal_task3.metrics.theta_final_rad = abs(e_target_theta(end));

Dreal_task3.metrics.ekf_rmse_m = sqrt(mean(e_ekf_pos.^2,'omitnan'));
Dreal_task3.metrics.ekf_max_m = max(e_ekf_pos,[],'omitnan');
Dreal_task3.metrics.ekf_theta_rmse_rad = sqrt(mean(e_ekf_theta.^2,'omitnan'));

Dreal_task3.metrics.imu_rmse_m = sqrt(mean(e_imu_pos.^2,'omitnan'));
Dreal_task3.metrics.imu_max_m = max(e_imu_pos,[],'omitnan');

Dreal_task3.metrics.raw_vicon_rmse_m = sqrt(mean(e_raw_vicon_pos.^2,'omitnan'));
Dreal_task3.metrics.raw_vicon_max_m = max(e_raw_vicon_pos,[],'omitnan');

Dreal_task3.metrics.vicon_loss_percent = 100 * mean(Dreal_task3.loss_flag > 0.5,'omitnan');
Dreal_task3.metrics.valid_vicon_percent = 100 * mean(Dreal_task3.vicon_valid > 0.5,'omitnan');
Dreal_task3.metrics.red_percent = 100 * mean(Dreal_task3.in_red > 0.5,'omitnan');
Dreal_task3.metrics.orange_percent = 100 * mean(Dreal_task3.in_orange > 0.5,'omitnan');

Dreal_task3.metrics.ekf_rmse_valid_m = rmseMasked(e_ekf_pos, validVicon);
Dreal_task3.metrics.ekf_rmse_lost_m = rmseMasked(e_ekf_pos, Dreal_task3.loss_flag > 0.5);
Dreal_task3.metrics.ekf_rmse_red_m = rmseMasked(e_ekf_pos, Dreal_task3.in_red > 0.5);
Dreal_task3.metrics.ekf_rmse_orange_m = rmseMasked(e_ekf_pos, Dreal_task3.in_orange > 0.5);

if ~isnan(Dreal_task3.metrics.imu_rmse_m) && Dreal_task3.metrics.imu_rmse_m > 0
    Dreal_task3.metrics.ekf_improvement_vs_imu_percent = ...
        100 * (1 - Dreal_task3.metrics.ekf_rmse_m / Dreal_task3.metrics.imu_rmse_m);
else
    Dreal_task3.metrics.ekf_improvement_vs_imu_percent = NaN;
end

Dreal_task3.metrics.max_v_m_s = max(abs(Dreal_task3.v),[],'omitnan');
Dreal_task3.metrics.max_omega_rad_s = max(abs(Dreal_task3.omega),[],'omitnan');
Dreal_task3.metrics.max_wheel_cmd_rad_s = max(abs(Dreal_task3.omega_wheels(:)),[],'omitnan');
Dreal_task3.metrics.max_wheel_measured_rad_s = max(abs(Dreal_task3.bot_wheelspeed(:)),[],'omitnan');

%% ============================================================
% SAVE
% ============================================================
save(outFile, 'Dreal_task3', 'Dreal_full');

fprintf('\nSaved:\n%s\n', fullfile(pwd, outFile));

fprintf('\n================ REAL TASK 3 SEGMENT METRICS ================\n');
disp(Dreal_task3.metrics);

fprintf('\nLoss intervals [s]:\n');
disp(Dreal_task3.loss_intervals);

fprintf('Red intervals [s]:\n');
disp(Dreal_task3.red_intervals);

fprintf('Orange intervals [s]:\n');
disp(Dreal_task3.orange_intervals);

fprintf('\nNext step: run plot_task3_real_vs_sim_from_extracted.m\n');

%% =======================================================================
% LOCAL FUNCTIONS
% =======================================================================

function source = findSimulationOutput(L)
    source = [];
    f = fieldnames(L);
    for i = 1:numel(f)
        obj = L.(f{i});
        if isa(obj, 'Simulink.SimulationOutput')
            source = obj;
            return;
        end
    end
    if isfield(L, 'out')
        source = L.out;
    end
end

function X = getSignal(source, L, names)
    X = [];

    for i = 1:numel(names)
        name = names{i};

        if isa(source, 'Simulink.SimulationOutput')
            try
                tmp = source.get(name);
                tmp = toNumeric(tmp);
                if ~isempty(tmp)
                    X = tmp;
                    return;
                end
            catch
            end

            try
                logsout = source.get('logsout');
                if ~isempty(logsout)
                    el = logsout.getElement(name);
                    tmp = toNumeric(el.Values);
                    if ~isempty(tmp)
                        X = tmp;
                        return;
                    end
                end
            catch
            end
        end

        if isstruct(source) && isfield(source, name)
            tmp = toNumeric(source.(name));
            if ~isempty(tmp)
                X = tmp;
                return;
            end
        end

        if isfield(L, name)
            tmp = toNumeric(L.(name));
            if ~isempty(tmp)
                X = tmp;
                return;
            end
        end
    end
end

function X = toNumeric(X)
    if isempty(X)
        return;
    end

    if isa(X, 'timeseries')
        X = X.Data;
    elseif isa(X, 'Simulink.SimulationData.Signal')
        X = X.Values;
        X = toNumeric(X);
    elseif isstruct(X)
        if isfield(X, 'signals') && isfield(X.signals, 'values')
            X = X.signals.values;
        elseif isfield(X, 'Data')
            X = X.Data;
        else
            X = [];
            return;
        end
    elseif ~isnumeric(X)
        X = [];
        return;
    end

    X = squeeze(X);
end

function X = toNxM(X, M, signalName)
    X = toNumeric(X);

    if isempty(X)
        error('Signal "%s" is empty. Expected N x %d.', signalName, M);
    end

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
        error('Signal "%s" has wrong size. Expected N x %d, got %s.', ...
            signalName, M, mat2str(size(X)));
    end
end

function y = toCol(y, signalName)
    y = toNumeric(y);

    if isempty(y)
        error('Signal "%s" is empty. Expected column vector.', signalName);
    end

    y = squeeze(y);

    if isrow(y)
        y = y.';
    end

    if size(y,2) > 1
        y = y(:,1);
    end
end

function X = fitRows(X, N)
    if isempty(X)
        X = NaN(N,1);
        return;
    end

    if size(X,1) == N
        return;
    elseif size(X,1) == 1
        X = repmat(X, N, 1);
    elseif size(X,1) > N
        X = X(1:N,:);
    else
        lastRow = X(end,:);
        X(end+1:N,:) = repmat(lastRow, N-size(X,1), 1);
    end
end

function [idxStart, reason] = detectTask3Start(t, q_target, Y_gt, q_goal_task2, ...
    targetPosTol, targetThetaTol, robotPosTol, task3_active, state_out)

    idxStart = [];
    reason = 'not_detected';

    if ~isempty(task3_active) && any(task3_active > 0.5)
        idxStart = find(task3_active > 0.5, 1, 'first');
        reason = 'task3_active signal';
        return;
    end

    % If state_out exists and has a clear transition, use it carefully.
    if ~isempty(state_out)
        uniqueStates = unique(state_out(~isnan(state_out)));
        if numel(uniqueStates) > 1
            dState = [0; diff(state_out)];
            idxChange = find(abs(dState) > 0.5, 1, 'last');
            if ~isempty(idxChange) && idxChange < numel(state_out)
                idxStart = idxChange + 1;
                reason = 'last state_out transition';
                return;
            end
        end
    end

    q_goal_task2 = q_goal_task2(:);

    % Method 1: q_target is close to Task 2 goal, then changes.
    targetPosDist = vecnorm(q_target(:,1:2) - q_goal_task2(1:2).', 2, 2);
    targetThetaErr = abs(wrapToPiLocal(q_target(:,3) - q_goal_task2(3)));

    maskTargetTask2 = targetPosDist < targetPosTol & targetThetaErr < targetThetaTol;

    if any(maskTargetTask2)
        lastTask2Target = find(maskTargetTask2, 1, 'last');
        idxStart = min(lastTask2Target + 1, size(q_target,1));
        reason = 'after last q_target close to Task 2 goal';
        return;
    end

    % Method 2: robot reaches near Task 2 goal.
    robotPosDist = vecnorm(Y_gt(:,1:2) - q_goal_task2(1:2).', 2, 2);
    maskRobotTask2 = robotPosDist < robotPosTol;

    if any(maskRobotTask2)
        lastNearTask2 = find(maskRobotTask2, 1, 'last');
        idxStart = min(lastNearTask2 + 1, size(Y_gt,1));
        reason = 'after robot last near Task 2 goal';
        return;
    end

    % Method 3: first target in orange/Task 3 side.
    idx = find(q_target(:,1) > 1.20, 1, 'first');
    if ~isempty(idx)
        idxStart = idx;
        reason = 'first q_target x > 1.20';
        return;
    end
end

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end

function omega_wheels = wheelsFromVOmega(v, omega, r, d)
    v = v(:);
    omega = omega(:);

    omega_L = (v - omega*d/2) / r;
    omega_R = (v + omega*d/2) / r;

    omega_wheels = [omega_L, omega_R];
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

function y = rmseMasked(e, mask)
    mask = mask(:);
    e = e(:);

    if isempty(mask) || ~any(mask)
        y = NaN;
    else
        y = sqrt(mean(e(mask).^2,'omitnan'));
    end
end
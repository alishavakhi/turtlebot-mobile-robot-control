clear; clc; close all;

%% ============================================================
% Export and compare ALL TASKS from realrobot_test2.mat and realrobot_test3.mat
%
% This script:
% 1) Loads Test 2 and Test 3 raw .mat files
% 2) Exports all useful signals
% 3) Splits data into Task 1, Task 2, Task 3 using q_target switching
% 4) Saves each task separately
% 5) Compares Test 2 vs Test 3 for each task
% 6) Recommends the better test for each task
%
% Output folder:
%   split_and_compare_all_tasks/
% ============================================================

%% --------- FIND SCRIPT FOLDER ---------
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    scriptFolder = pwd;
else
    scriptFolder = fileparts(scriptPath);
end

fprintf('\nScript folder:\n%s\n', scriptFolder);

%% --------- USER SETTINGS ---------
tests = {
    'realrobot_test2.mat', 'test2', 'Test 2';
    'realrobot_test3.mat', 'test3', 'Test 3';
};

t_start_first_task = 0.3200;     % remove initial transient only before Task 1
target_change_tol  = 1e-4;       % q_target change detection tolerance
minSamplesPerTask  = 5;

% Reference priority
% If ground-truth exists, use it. Otherwise use Y_viscon.
referenceCandidates = {
    'Y_viscon_ground_trith'
    'Y_vicon_ground_truth'
    'Y_viscon_ground_truth'
    'Y_viscon'
    'Y_vicon'
};

outputRoot = fullfile(scriptFolder, 'split_and_compare_all_tasks');

if ~exist(outputRoot, 'dir')
    mkdir(outputRoot);
end

for taskID = 1:3
    taskFolder = fullfile(outputRoot, sprintf('task%d', taskID));
    if ~exist(taskFolder, 'dir')
        mkdir(taskFolder);
    end
end

plotsFolder = fullfile(outputRoot, 'comparison_plots');
if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

wrap = @(a) atan2(sin(a), cos(a));

%% ============================================================
% PART 1 — EXPORT AND SPLIT BOTH TESTS
% ============================================================

for k = 1:size(tests,1)

    rawFileName = tests{k,1};
    testPrefix  = tests{k,2};
    testLabel   = tests{k,3};

    rawFile = fullfile(scriptFolder, rawFileName);

    fprintf('\n============================================================\n');
    fprintf('Processing %s\n', testLabel);
    fprintf('Raw file: %s\n', rawFile);
    fprintf('============================================================\n');

    if ~isfile(rawFile)
        warning('File not found: %s. Skipping.', rawFile);
        continue;
    end

    %% Load raw file
    S = load(rawFile);

    if isfield(S,'out')
        outRaw = S.out;
    else
        vars = fieldnames(S);
        outRaw = [];

        for i = 1:numel(vars)
            if isa(S.(vars{i}), 'Simulink.SimulationOutput')
                outRaw = S.(vars{i});
                fprintf('Found SimulationOutput variable: %s\n', vars{i});
                break;
            end
        end

        if isempty(outRaw)
            warning('No Simulink output object found in %s. Skipping.', rawFileName);
            continue;
        end
    end

    %% Export all available signals
    sigNames = outRaw.who;

    fprintf('\nAvailable signals inside out:\n');
    disp(sigNames);

    D_all = struct();

    for i = 1:numel(sigNames)

        name = sigNames{i};

        try
            sig = outRaw.(name);
            X = getSignalData(sig);

            switch name
                case {'q_est','q_est_imu','q_target','Y_viscon', ...
                      'Y_vicon','Y_viscon_ground_trith', ...
                      'Y_vicon_ground_truth','Y_viscon_ground_truth'}
                    X = forceCols(X, 3);

                case {'omega_wheels','bot_wheelspeed','u_actual'}
                    X = forceCols(X, 2);

                case {'v','omega','t','tout'}
                    X = X(:);

                otherwise
                    if isrow(X)
                        X = X.';
                    end
            end

            D_all.(name) = X;

            fprintf('Saved signal %-35s size: ', name);
            disp(size(X));

        catch ME
            fprintf('Could not export signal %-30s : %s\n', name, ME.message);
        end
    end

    %% Time vector
    if isfield(D_all,'tout')
        t_abs = D_all.tout(:);
    elseif isfield(D_all,'t')
        t_abs = D_all.t(:);
    elseif isprop(outRaw,'tout')
        t_abs = outRaw.tout(:);
    else
        warning('No time vector found for %s. Skipping.', testLabel);
        continue;
    end

    D_all.t = t_abs;

    %% q_target check
    if ~isfield(D_all,'q_target')
        warning('q_target not found for %s. Cannot split tasks.', testLabel);
        continue;
    end

    q_target = forceCols(D_all.q_target, 3);

    N = min(length(t_abs), size(q_target,1));
    t_abs = t_abs(1:N);
    q_target = q_target(1:N,:);

    %% Detect task segments from q_target switching
    idx_global_start = find(t_abs >= t_start_first_task, 1, 'first');
    if isempty(idx_global_start)
        idx_global_start = 1;
    end

    dq = vecnorm(diff(q_target,1,1), 2, 2);
    changeIdx = find(dq > target_change_tol) + 1;

    changeIdx = changeIdx(changeIdx > idx_global_start);

    segStarts = [idx_global_start; changeIdx(:)];
    segEnds   = [changeIdx(:)-1; N];

    validSeg = (segEnds - segStarts + 1) >= minSamplesPerTask;
    segStarts = segStarts(validSeg);
    segEnds   = segEnds(validSeg);

    numTasks = min(3, numel(segStarts));

    fprintf('\nDetected task segments for %s:\n', testLabel);

    for taskID = 1:numTasks
        i1 = segStarts(taskID);
        i2 = segEnds(taskID);

        fprintf('Task %d: idx %d to %d | t %.3f to %.3f | duration %.3f s | q_target = [%.4f %.4f %.4f]\n', ...
            taskID, i1, i2, t_abs(i1), t_abs(i2), t_abs(i2)-t_abs(i1), ...
            q_target(i1,1), q_target(i1,2), q_target(i1,3));
    end

    %% Export each task
    for taskID = 1:numTasks

        idx_start = segStarts(taskID);
        idx_end   = segEnds(taskID);

        D_task = struct();
        fields = fieldnames(D_all);

        for i = 1:numel(fields)
            f = fields{i};
            X = D_all.(f);

            if isnumeric(X)
                if size(X,1) >= idx_end
                    D_task.(f) = X(idx_start:idx_end,:);
                else
                    D_task.(f) = X;
                end
            else
                D_task.(f) = X;
            end
        end

        % Reset time to zero
        if isfield(D_task,'t')
            D_task.t = D_task.t - D_task.t(1);
        end

        if isfield(D_task,'tout')
            D_task.tout = D_task.tout - D_task.tout(1);
        end

        % Store original interval and goal
        D_task.t_abs_start = t_abs(idx_start);
        D_task.t_abs_end   = t_abs(idx_end);
        D_task.idx_start_original = idx_start;
        D_task.idx_end_original   = idx_end;
        D_task.q_goal = D_task.q_target(1,:).';

        taskFolder = fullfile(outputRoot, sprintf('task%d', taskID));
        saveName = sprintf('%s_task%d_all_outputs.mat', testPrefix, taskID);
        savePath = fullfile(taskFolder, saveName);

        D_task1 = D_task; %#ok<NASGU>
        save(savePath, 'D_task1');

        fprintf('\nSaved %s Task %d file:\n%s\n', testLabel, taskID, savePath);
    end
end

%% ============================================================
% PART 2 — COMPARE TEST 2 VS TEST 3 FOR EACH TASK
% ============================================================

fprintf('\n============================================================\n');
fprintf('Comparing Test 2 vs Test 3 for each task\n');
fprintf('============================================================\n');

AllResults = table();
BestByTask = table();

for taskID = 1:3

    taskResults = table();

    for k = 1:size(tests,1)

        testPrefix = tests{k,2};
        testLabel  = tests{k,3};

        taskFolder = fullfile(outputRoot, sprintf('task%d', taskID));
        fileName = sprintf('%s_task%d_all_outputs.mat', testPrefix, taskID);
        filePath = fullfile(taskFolder, fileName);

        if ~isfile(filePath)
            warning('%s not found. Skipping.', filePath);
            continue;
        end

        S = load(filePath);
        D = S.D_task1;

        if isfield(D,'t')
            t = D.t(:);
        elseif isfield(D,'tout')
            t = D.tout(:);
        else
            warning('No time vector for %s Task %d. Skipping.', testLabel, taskID);
            continue;
        end

        if ~isfield(D,'q_est') || ~isfield(D,'q_est_imu') || ~isfield(D,'q_target')
            warning('Missing q_est/q_est_imu/q_target for %s Task %d. Skipping.', testLabel, taskID);
            continue;
        end

        q_est = forceCols(D.q_est, 3);
        q_imu = forceCols(D.q_est_imu, 3);
        q_goal = D.q_goal;

        [Yref, refName, hasRef] = chooseReference(D, referenceCandidates);

        if ~hasRef
            warning('No valid reference pose found for %s Task %d.', testLabel, taskID);
            continue;
        end

        N = min([length(t), size(q_est,1), size(q_imu,1), size(Yref,1)]);
        t = t(1:N);
        q_est = q_est(1:N,:);
        q_imu = q_imu(1:N,:);
        Yref = Yref(1:N,:);

        validRef = ~(abs(Yref(:,1)) < 1e-9 & ...
                     abs(Yref(:,2)) < 1e-9 & ...
                     abs(Yref(:,3)) < 1e-9);

        if ~any(validRef)
            warning('Reference has no valid samples for %s Task %d.', testLabel, taskID);
            continue;
        end

        t_valid = t(validRef);
        Yv = Yref(validRef,:);
        q_est_valid = q_est(validRef,:);
        q_imu_valid = q_imu(validRef,:);

        %% Final performance using reference
        Y_end = Yv(end,:);

        final_ref_pos_err = norm(Y_end(1:2).' - q_goal(1:2));
        final_ref_th_err  = abs(wrap(q_goal(3) - Y_end(3)));

        %% Final performance using q_est
        q_end = q_est(end,:);

        final_qest_pos_err = norm(q_end(1:2).' - q_goal(1:2));
        final_qest_th_err  = abs(wrap(q_goal(3) - q_end(3)));

        %% q_est vs reference
        e_qest_xy = vecnorm(q_est_valid(:,1:2) - Yv(:,1:2), 2, 2);
        e_qest_th = abs(wrap(Yv(:,3) - q_est_valid(:,3)));

        mean_qest_ref_xy = mean(e_qest_xy,'omitnan');
        max_qest_ref_xy  = max(e_qest_xy,[],'omitnan');
        mean_qest_ref_th = mean(e_qest_th,'omitnan');
        max_qest_ref_th  = max(e_qest_th,[],'omitnan');

        %% q_est_imu vs reference
        e_imu_xy = vecnorm(q_imu_valid(:,1:2) - Yv(:,1:2), 2, 2);
        e_imu_th = abs(wrap(Yv(:,3) - q_imu_valid(:,3)));

        mean_imu_ref_xy = mean(e_imu_xy,'omitnan');
        max_imu_ref_xy  = max(e_imu_xy,[],'omitnan');
        mean_imu_ref_th = mean(e_imu_th,'omitnan');
        max_imu_ref_th  = max(e_imu_th,[],'omitnan');

        %% Reference smoothness
        ref_step = vecnorm(diff(Yv(:,1:2)), 2, 2);
        mean_ref_step = mean(ref_step,'omitnan');
        max_ref_step  = max(ref_step,[],'omitnan');

        %% Command / wheel metrics
        max_v = NaN;
        max_omega = NaN;
        max_cmd_wheel = NaN;
        max_meas_wheel = NaN;

        if isfield(D,'v')
            max_v = max(abs(D.v(:)),[],'omitnan');
        end

        if isfield(D,'omega')
            max_omega = max(abs(D.omega(:)),[],'omitnan');
        end

        if isfield(D,'omega_wheels')
            max_cmd_wheel = max(abs(D.omega_wheels(:)),[],'omitnan');
        end

        if isfield(D,'bot_wheelspeed')
            max_meas_wheel = max(abs(D.bot_wheelspeed(:)),[],'omitnan');
        end

        duration = t(end) - t(1);

        %% Score
        % Lower is better.
        % Main priority: final reference position error and orientation error.
        % Secondary: localization error and reference jumps.
        score = ...
            10.0 * final_ref_pos_err + ...
            0.50 * final_ref_th_err + ...
            5.00 * mean_qest_ref_xy + ...
            2.00 * max_ref_step + ...
            0.10 * mean_imu_ref_xy;

        row = table( ...
            taskID, string(testLabel), string(refName), duration, ...
            q_goal(1), q_goal(2), q_goal(3), ...
            final_ref_pos_err, final_ref_th_err, rad2deg(final_ref_th_err), ...
            final_qest_pos_err, final_qest_th_err, rad2deg(final_qest_th_err), ...
            mean_qest_ref_xy, max_qest_ref_xy, mean_qest_ref_th, rad2deg(mean_qest_ref_th), ...
            mean_imu_ref_xy, max_imu_ref_xy, mean_imu_ref_th, rad2deg(mean_imu_ref_th), ...
            mean_ref_step, max_ref_step, ...
            max_v, max_omega, max_cmd_wheel, max_meas_wheel, ...
            score, ...
            'VariableNames', { ...
            'Task','Test','ReferenceSignal','Duration_s', ...
            'Goal_x','Goal_y','Goal_theta', ...
            'FinalRefPosErr_m','FinalRefThetaErr_rad','FinalRefThetaErr_deg', ...
            'FinalQestPosErr_m','FinalQestThetaErr_rad','FinalQestThetaErr_deg', ...
            'MeanQestRefXY_m','MaxQestRefXY_m','MeanQestRefTheta_rad','MeanQestRefTheta_deg', ...
            'MeanIMURefXY_m','MaxIMURefXY_m','MeanIMURefTheta_rad','MeanIMURefTheta_deg', ...
            'MeanRefStep_m','MaxRefStep_m', ...
            'Max_v','Max_omega','MaxCommandedWheelSpeed','MaxMeasuredWheelSpeed', ...
            'Score'} ...
        );

        taskResults = [taskResults; row]; %#ok<AGROW>
        AllResults = [AllResults; row]; %#ok<AGROW>

        %% Plots for each test/task
        fig = figure('Name', sprintf('%s Task %d XY', testLabel, taskID), 'Color', 'w');
        plot(Yv(:,1), Yv(:,2), 'LineWidth', 1.8); hold on;
        plot(q_est_valid(:,1), q_est_valid(:,2), '--', 'LineWidth', 1.4);
        plot(q_imu_valid(:,1), q_imu_valid(:,2), ':', 'LineWidth', 1.4);
        plot(q_goal(1), q_goal(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
        plot(Yv(1,1), Yv(1,2), 'ko', 'MarkerSize', 7, 'LineWidth', 1.5);
        grid on; axis equal;
        xlabel('x [m]');
        ylabel('y [m]');
        title(sprintf('%s - Task %d XY comparison', testLabel, taskID));
        legend(refName, 'EKF/localizer estimate', 'Odometry/IMU estimate', 'Goal', 'Start', ...
            'Location', 'bestoutside');

        saveas(fig, fullfile(plotsFolder, sprintf('%s_task%d_xy.png', char(testLabel), taskID)));

        fig = figure('Name', sprintf('%s Task %d errors', testLabel, taskID), 'Color', 'w');
        plot(t_valid, e_qest_xy, 'LineWidth', 1.6); hold on;
        plot(t_valid, e_imu_xy, 'LineWidth', 1.3);
        grid on;
        xlabel('Time [s]');
        ylabel('XY error w.r.t. reference [m]');
        title(sprintf('%s - Task %d localization/odometry error', testLabel, taskID));
        legend('q_{est} vs reference', 'q_{est,imu} vs reference', 'Location', 'best');

        saveas(fig, fullfile(plotsFolder, sprintf('%s_task%d_errors.png', char(testLabel), taskID)));
    end

    %% Best for this task
    if height(taskResults) > 0
        [~, bestIdx] = min(taskResults.Score);
        bestRow = taskResults(bestIdx,:);

        BestByTask = [BestByTask; bestRow]; %#ok<AGROW>

        fprintf('\nBest test for Task %d: %s\n', taskID, bestRow.Test);
        fprintf('Score = %.4f | FinalRefPosErr = %.4f m | FinalRefThetaErr = %.2f deg\n', ...
            bestRow.Score, bestRow.FinalRefPosErr_m, bestRow.FinalRefThetaErr_deg);
    end
end

%% Save tables
allResultsPath = fullfile(outputRoot, 'all_tasks_test2_vs_test3_metrics.csv');
bestPath       = fullfile(outputRoot, 'best_test_by_task.csv');

writetable(AllResults, allResultsPath);
writetable(BestByTask, bestPath);

fprintf('\n============================================================\n');
fprintf('All comparison results:\n');
disp(AllResults);

fprintf('\nBest test by task:\n');
disp(BestByTask(:, {'Task','Test','FinalRefPosErr_m','FinalRefThetaErr_deg','MeanQestRefXY_m','MeanIMURefXY_m','Score'}));

fprintf('\nSaved all results table:\n%s\n', allResultsPath);
fprintf('Saved best-by-task table:\n%s\n', bestPath);

%% ============================================================
% Helper functions
% ============================================================

function X = getSignalData(sig)

    if isa(sig,'timeseries')
        X = squeeze(sig.Data);

    elseif isnumeric(sig)
        X = squeeze(sig);

    elseif isstruct(sig) && isfield(sig,'signals')
        X = squeeze(sig.signals.values);

    elseif isa(sig,'Simulink.SimulationData.Signal')
        X = squeeze(sig.Values.Data);

    else
        error('Unknown signal format.');
    end
end

function X = forceCols(X, ncols)

    X = squeeze(X);

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Expected %d columns, got size %dx%d.', ncols, size(X,1), size(X,2));
    end
end

function [Yref, refName, hasRef] = chooseReference(D, referenceCandidates)

    Yref = [];
    refName = "none";
    hasRef = false;

    for i = 1:numel(referenceCandidates)

        name = referenceCandidates{i};

        if isfield(D,name)
            try
                X = forceCols(D.(name), 3);
                Yref = X;
                refName = string(name);
                hasRef = true;
                return;
            catch
            end
        end
    end
end

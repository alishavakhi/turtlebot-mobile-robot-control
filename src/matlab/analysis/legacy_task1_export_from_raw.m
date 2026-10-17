clear; clc; close all;

%% ============================================================
% Export and compare Task 1 from realrobot_test2.mat and realrobot_test3.mat
% The script:
% 1) reads both raw real robot .mat files from the script folder
% 2) exports Task 1 data from each file
% 3) saves test2_task1_all_outputs.mat and test3_task1_all_outputs.mat
% 4) compares both tests using Vicon/ROS measurement, q_est, q_est_imu, inputs
% 5) saves comparison table and plots
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
    'realrobot_test2.mat', 'test2_task1_all_outputs.mat', 'Test 2';
    'realrobot_test3.mat', 'test3_task1_all_outputs.mat', 'Test 3';
};

q_goal_task1 = [-1.60; -1.10; 0];
dist_threshold = 0.05;
t_start_abs = 0.3200;       % remove initial localization transient
target_tol = 1e-3;          % tolerance for detecting q_target = task 1 goal

plotsFolder = fullfile(scriptFolder, 'compare_test2_test3_plots');
if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

wrap = @(a) atan2(sin(a), cos(a));

%% ============================================================
% PART 1 — EXPORT TASK 1 FOR BOTH TESTS
% ============================================================

for k = 1:size(tests,1)

    rawFileName = tests{k,1};
    saveName    = tests{k,2};
    testLabel   = tests{k,3};

    rawFile  = fullfile(scriptFolder, rawFileName);
    savePath = fullfile(scriptFolder, saveName);

    fprintf('\n============================================================\n');
    fprintf('Processing %s\n', testLabel);
    fprintf('Raw file: %s\n', rawFileName);
    fprintf('============================================================\n');

    if ~isfile(rawFile)
        warning('File not found: %s. Skipping this test.', rawFile);
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
            warning('No variable named out or Simulink.SimulationOutput found in %s.', rawFileName);
            continue;
        end
    end

    %% List and export all available signals
    sigNames = outRaw.who;

    fprintf('\nAvailable signals inside out:\n');
    disp(sigNames);

    D_all = struct();

    for i = 1:numel(sigNames)
        name = sigNames{i};

        try
            sig = outRaw.(name);
            X = getSignalData(sig);

            % Fix known signal orientations
            switch name
                case {'q_est','q_est_imu','q_target','Y_viscon', ...
                      'Y_vicon','Y_viscon_ground_trith','Y_vicon_ground_truth', ...
                      'Y_viscon_ground_truth'}
                    X = tryForceCols(X, 3);

                case {'omega_wheels','bot_wheelspeed','u_actual'}
                    X = tryForceCols(X, 2);

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
            fprintf('Could not export signal %-25s : %s\n', name, ME.message);
        end
    end

    %% Time vector
    if isfield(D_all,'tout')
        t = D_all.tout(:);
    elseif isfield(D_all,'t')
        t = D_all.t(:);
    elseif isprop(outRaw,'tout')
        t = outRaw.tout(:);
    else
        warning('No time vector found for %s. Skipping.', testLabel);
        continue;
    end

    D_all.t = t;
    N = length(t);

    %% Detect Task 1 interval
    idx_start = find(t >= t_start_abs, 1, 'first');
    if isempty(idx_start)
        idx_start = 1;
    end

    idx_end = [];

    % Best method: use q_target change if q_target exists
    if isfield(D_all,'q_target')
        try
            qtar = forcePose(D_all.q_target);
            M = min(size(qtar,1), N);
            qtar = qtar(1:M,:);

            is_task1_target = ...
                abs(qtar(:,1) - q_goal_task1(1)) < target_tol & ...
                abs(qtar(:,2) - q_goal_task1(2)) < target_tol & ...
                abs(wrap(qtar(:,3) - q_goal_task1(3))) < target_tol;

            candidates = find((1:M)' >= idx_start & is_task1_target);

            if ~isempty(candidates)
                idx_end = candidates(end);
                fprintf('\nTask 1 end detected from q_target switching.\n');
            end
        catch
            idx_end = [];
        end
    end

    % Fallback method: first time q_est reaches distance threshold
    if isempty(idx_end)
        if isfield(D_all,'q_est')
            qsplit = forcePose(D_all.q_est);
        elseif isfield(D_all,'Y_viscon')
            qsplit = forcePose(D_all.Y_viscon);
        else
            warning('No q_est or Y_viscon found. Cannot split %s.', testLabel);
            continue;
        end

        M = min(size(qsplit,1), N);
        qsplit = qsplit(1:M,:);

        dist_to_goal = sqrt((qsplit(:,1) - q_goal_task1(1)).^2 + ...
                            (qsplit(:,2) - q_goal_task1(2)).^2);

        candidates = find((1:M)' >= idx_start & dist_to_goal <= dist_threshold);

        if isempty(candidates)
            warning('%s never reached 5 cm threshold. Using end of data.', testLabel);
            idx_end = M;
        else
            idx_end = candidates(1);
            fprintf('\nTask 1 end detected from 5 cm position threshold.\n');
        end
    end

    fprintf('\nTask 1 split for %s:\n', testLabel);
    fprintf('idx_start = %d, t_start = %.3f s\n', idx_start, t(idx_start));
    fprintf('idx_end   = %d, t_end   = %.3f s\n', idx_end, t(idx_end));
    fprintf('Task 1 duration after transient = %.3f s\n', t(idx_end) - t(idx_start));

    %% Build cropped D_task1
    D_task1 = struct();
    fields = fieldnames(D_all);

    for i = 1:numel(fields)
        f = fields{i};
        X = D_all.(f);

        if isnumeric(X)
            if size(X,1) >= idx_end
                D_task1.(f) = X(idx_start:idx_end,:);
            else
                D_task1.(f) = X;
            end
        else
            D_task1.(f) = X;
        end
    end

    %% Reset time to zero
    if isfield(D_task1,'t')
        D_task1.t = D_task1.t - D_task1.t(1);
    end

    if isfield(D_task1,'tout')
        D_task1.tout = D_task1.tout - D_task1.tout(1);
    end

    %% Print important signals
    fprintf('\nImportant Task 1 signals for %s:\n', testLabel);

    importantNames = {
        'q_est'
        'Y_viscon'
        'Y_viscon_ground_trith'
        'Y_vicon_ground_truth'
        'Y_viscon_ground_truth'
        'q_est_imu'
        'q_target'
        'bot_accelerpmeter'
        'bot_gyroscope'
        'bot_wheelspeed'
        'v'
        'omega'
        'omega_wheels'
        'u_actual'
        't'
        'tout'
    };

    for i = 1:numel(importantNames)
        name = importantNames{i};

        if isfield(D_task1,name)
            fprintf('%-35s size: ', name);
            disp(size(D_task1.(name)));

            if isnumeric(D_task1.(name))
                fprintf('first rows/value:\n');
                disp(D_task1.(name)(1:min(3,size(D_task1.(name),1)),:));
            end
        end
    end

    %% Final errors based on q_est and Y_viscon
    if isfield(D_task1,'q_est')
        q_est = forcePose(D_task1.q_est);
        q_end = q_est(end,:);

        pos_err = norm(q_end(1:2).' - q_goal_task1(1:2));
        th_err  = abs(wrap(q_goal_task1(3) - q_end(3)));

        fprintf('\n%s final q_est:\n', testLabel);
        fprintf('[%.4f %.4f %.4f]\n', q_end(1), q_end(2), q_end(3));
        fprintf('q_est position error = %.4f m\n', pos_err);
        fprintf('q_est theta error    = %.4f rad = %.2f deg\n', th_err, rad2deg(th_err));
    end

    if isfield(D_task1,'Y_viscon')
        Yv = forcePose(D_task1.Y_viscon);

        valid = ~(abs(Yv(:,1)) < 1e-9 & abs(Yv(:,2)) < 1e-9 & abs(Yv(:,3)) < 1e-9);

        if any(valid)
            Yv_valid = Yv(valid,:);
            Yv_end = Yv_valid(end,:);

            pos_err_vicon = norm(Yv_end(1:2).' - q_goal_task1(1:2));
            th_err_vicon  = abs(wrap(q_goal_task1(3) - Yv_end(3)));

            fprintf('\n%s final valid Y_viscon:\n', testLabel);
            fprintf('[%.4f %.4f %.4f]\n', Yv_end(1), Yv_end(2), Yv_end(3));
            fprintf('Y_viscon position error = %.4f m\n', pos_err_vicon);
            fprintf('Y_viscon theta error    = %.4f rad = %.2f deg\n', th_err_vicon, rad2deg(th_err_vicon));
        end
    end

    %% Save cropped file
    save(savePath, 'D_task1');

    fprintf('\nSaved exported Task 1 file:\n%s\n', savePath);
end

%% ============================================================
% PART 2 — COMPARE EXPORTED TEST 2 AND TEST 3
% ============================================================

fprintf('\n============================================================\n');
fprintf('Comparing exported Task 1 files\n');
fprintf('============================================================\n');

Results = table();

for k = 1:size(tests,1)

    saveName  = tests{k,2};
    testLabel = tests{k,3};
    filePath  = fullfile(scriptFolder, saveName);

    if ~isfile(filePath)
        warning('%s not found. Skipping comparison.', saveName);
        continue;
    end

    S = load(filePath);
    D = S.D_task1;

    if isfield(D,'t')
        t = D.t(:);
    elseif isfield(D,'tout')
        t = D.tout(:);
    else
        warning('No time vector in %s. Skipping.', saveName);
        continue;
    end

    q_est = forcePose(D.q_est);
    q_imu = forcePose(D.q_est_imu);

    % Choose external reference:
    % Prefer ground-truth-like signals only if they are N x 3.
    [Yref, refName] = chooseReference(D);

    N = min([length(t), size(q_est,1), size(q_imu,1), size(Yref,1)]);
    t = t(1:N);
    q_est = q_est(1:N,:);
    q_imu = q_imu(1:N,:);
    Yref = Yref(1:N,:);

    validRef = ~(abs(Yref(:,1)) < 1e-9 & abs(Yref(:,2)) < 1e-9 & abs(Yref(:,3)) < 1e-9);

    if ~any(validRef)
        warning('%s has no valid reference samples.', testLabel);
        continue;
    end

    t_valid = t(validRef);
    Yref_valid = Yref(validRef,:);
    q_est_valid = q_est(validRef,:);
    q_imu_valid = q_imu(validRef,:);

    %% Final real measured performance
    Y_end = Yref_valid(end,:);

    final_pos_err = norm(Y_end(1:2).' - q_goal_task1(1:2));
    final_th_err  = abs(wrap(q_goal_task1(3) - Y_end(3)));

    %% q_est vs reference
    e_est_xy = vecnorm(q_est_valid(:,1:2) - Yref_valid(:,1:2), 2, 2);
    e_est_th = abs(wrap(Yref_valid(:,3) - q_est_valid(:,3)));

    mean_est_xy = mean(e_est_xy, 'omitnan');
    max_est_xy  = max(e_est_xy, [], 'omitnan');
    mean_est_th = mean(e_est_th, 'omitnan');

    %% q_est_imu vs reference
    e_imu_xy = vecnorm(q_imu_valid(:,1:2) - Yref_valid(:,1:2), 2, 2);
    e_imu_th = abs(wrap(Yref_valid(:,3) - q_imu_valid(:,3)));

    mean_imu_xy = mean(e_imu_xy, 'omitnan');
    max_imu_xy  = max(e_imu_xy, [], 'omitnan');
    mean_imu_th = mean(e_imu_th, 'omitnan');

    %% Reference smoothness / jumps
    ref_step = vecnorm(diff(Yref_valid(:,1:2)), 2, 2);
    mean_ref_step = mean(ref_step, 'omitnan');
    max_ref_step  = max(ref_step, [], 'omitnan');

    %% Control and wheel speed information
    if isfield(D,'v')
        max_v = max(abs(D.v(:)), [], 'omitnan');
    else
        max_v = NaN;
    end

    if isfield(D,'omega')
        max_omega = max(abs(D.omega(:)), [], 'omitnan');
    else
        max_omega = NaN;
    end

    if isfield(D,'omega_wheels')
        ow = D.omega_wheels;
        max_cmd_wheel = max(abs(ow(:)), [], 'omitnan');
    else
        max_cmd_wheel = NaN;
    end

    if isfield(D,'bot_wheelspeed')
        bw = D.bot_wheelspeed;
        max_meas_wheel = max(abs(bw(:)), [], 'omitnan');
    else
        max_meas_wheel = NaN;
    end

    duration = t(end) - t(1);

    %% Save results row
    Results = [Results; table( ...
        string(testLabel), string(refName), duration, ...
        final_pos_err, final_th_err, rad2deg(final_th_err), ...
        mean_est_xy, max_est_xy, mean_est_th, rad2deg(mean_est_th), ...
        mean_imu_xy, max_imu_xy, mean_imu_th, rad2deg(mean_imu_th), ...
        mean_ref_step, max_ref_step, ...
        max_v, max_omega, max_cmd_wheel, max_meas_wheel, ...
        'VariableNames', { ...
        'Test','ReferenceSignal','Duration_s', ...
        'FinalRefPosErr_m','FinalRefThetaErr_rad','FinalRefThetaErr_deg', ...
        'Mean_qest_Ref_XY_m','Max_qest_Ref_XY_m','Mean_qest_Ref_Theta_rad','Mean_qest_Ref_Theta_deg', ...
        'Mean_IMU_Ref_XY_m','Max_IMU_Ref_XY_m','Mean_IMU_Ref_Theta_rad','Mean_IMU_Ref_Theta_deg', ...
        'MeanRefStep_m','MaxRefStep_m', ...
        'Max_v','Max_omega','MaxCommandedWheelSpeed','MaxMeasuredWheelSpeed'} ...
    )];

    %% Plot XY comparison
    fig = figure('Name', [testLabel ' XY comparison'], 'Color', 'w');
    plot(Yref_valid(:,1), Yref_valid(:,2), 'LineWidth', 1.8); hold on;
    plot(q_est_valid(:,1), q_est_valid(:,2), '--', 'LineWidth', 1.4);
    plot(q_imu_valid(:,1), q_imu_valid(:,2), ':', 'LineWidth', 1.4);
    plot(q_goal_task1(1), q_goal_task1(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(Yref_valid(1,1), Yref_valid(1,2), 'ko', 'MarkerSize', 7, 'LineWidth', 1.5);
    grid on; axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    title([testLabel ' - Task 1 XY comparison']);
    legend(refName, 'EKF/localizer estimate', 'Odometry/IMU estimate', 'Goal', 'Start', ...
           'Location', 'bestoutside');

    saveas(fig, fullfile(plotsFolder, [char(testLabel) '_xy_comparison.png']));

    %% Plot estimation errors
    fig = figure('Name', [testLabel ' estimation errors'], 'Color', 'w');
    plot(t_valid, e_est_xy, 'LineWidth', 1.6); hold on;
    plot(t_valid, e_imu_xy, 'LineWidth', 1.3);
    grid on;
    xlabel('Time [s]');
    ylabel('Position error w.r.t. reference [m]');
    title([testLabel ' - Localization and odometry error']);
    legend('q_{est} vs reference', 'q_{est,imu} vs reference', 'Location', 'best');

    saveas(fig, fullfile(plotsFolder, [char(testLabel) '_estimation_errors.png']));
end

%% Print and save comparison table
disp(Results);

comparisonPath = fullfile(scriptFolder, 'compare_test2_test3_task1_metrics.csv');
writetable(Results, comparisonPath);

fprintf('\nSaved comparison table:\n%s\n', comparisonPath);

%% Simple automatic recommendation
if height(Results) >= 2
    % Lower score is better.
    % Main priorities: final position error, q_est/reference agreement, Vicon smoothness.
    score = ...
        10 * Results.FinalRefPosErr_m + ...
        5  * Results.Mean_qest_Ref_XY_m + ...
        2  * Results.MaxRefStep_m + ...
        0.1 * Results.FinalRefThetaErr_rad;

    [~, bestIdx] = min(score);

    fprintf('\nRecommended test for report based on the current metrics:\n');
    fprintf('%s\n', Results.Test(bestIdx));
    fprintf('Reason: lower combined position error, localization error, and reference jumps.\n');
end

%% ============================================================
% Local helper functions
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

function X = tryForceCols(X, ncols)

    X = squeeze(X);

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    % If it cannot be forced, keep original and let later logic decide.
    if size(X,2) ~= ncols
        if isrow(X)
            X = X.';
        end
    end
end

function Q = forcePose(X)

    X = squeeze(X);

    if size(X,2) ~= 3 && size(X,1) == 3
        X = X.';
    end

    if size(X,2) ~= 3
        error('Pose signal is not N x 3.');
    end

    Q = X;
end

function [Yref, refName] = chooseReference(D)

    candidates = {
        'Y_vicon_ground_truth'
        'Y_viscon_ground_truth'
        'Y_viscon_ground_trith'
        'Y_viscon'
        'Y_vicon'
    };

    for i = 1:numel(candidates)
        name = candidates{i};

        if isfield(D, name)
            X = D.(name);

            try
                X = forcePose(X);

                if size(X,2) == 3
                    Yref = X;
                    refName = name;
                    return;
                end
            catch
                % Not a pose signal. Try next.
            end
        end
    end

    error('No valid N x 3 reference pose signal found.');
end
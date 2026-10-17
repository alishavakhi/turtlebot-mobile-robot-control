clear; clc; close all;

%% ============================================================
% Split RAW real robot data into Task 1, Task 2, Task 3
% Based on q_target switching.
%
% Output files are saved in the SAME folder as this script:
%   test3_task1_all_outputs.mat
%   test3_task2_all_outputs.mat
%   test3_task3_all_outputs.mat
%   test3_3tasks_summary.csv
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
rawFileName = 'realrobot_test3.mat';   % change to realrobot_test2.mat if needed
rawFile = fullfile(scriptFolder, rawFileName);

testPrefix = 'test3';                  % output prefix

t_start_first_task = 0.3200;           % remove initial localization transient only before Task 1
target_change_tol = 1e-4;              % tolerance for detecting q_target change

% Reference priority for performance comparison
referenceCandidates = {
    'Y_viscon_ground_trith'
    'Y_vicon_ground_truth'
    'Y_viscon_ground_truth'
    'Y_viscon'
    'Y_vicon'
};

%% --------- LOAD RAW FILE ---------
if ~isfile(rawFile)
    error('Raw file not found:\n%s', rawFile);
end

S = load(rawFile);

fprintf('\nLoaded raw file:\n%s\n', rawFile);
fprintf('\nVariables inside raw file:\n');
disp(fieldnames(S));

%% --------- FIND SIMULINK OUTPUT OBJECT ---------
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
        error('No variable named "out" or Simulink.SimulationOutput found.');
    end
end

%% --------- EXPORT ALL SIGNALS ---------
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

%% --------- TIME VECTOR ---------
if isfield(D_all,'tout')
    t_abs = D_all.tout(:);
elseif isfield(D_all,'t')
    t_abs = D_all.t(:);
elseif isprop(outRaw,'tout')
    t_abs = outRaw.tout(:);
else
    error('No time vector found.');
end

D_all.t = t_abs;

%% --------- CHECK q_target ---------
if ~isfield(D_all,'q_target')
    error('q_target not found. Cannot split tasks automatically.');
end

q_target = forceCols(D_all.q_target, 3);

N = min(length(t_abs), size(q_target,1));
t_abs = t_abs(1:N);
q_target = q_target(1:N,:);

%% --------- DETECT TASK SEGMENTS FROM q_target SWITCHING ---------

idx_global_start = find(t_abs >= t_start_first_task, 1, 'first');
if isempty(idx_global_start)
    idx_global_start = 1;
end

% Detect changes in q_target
dq = vecnorm(diff(q_target,1,1), 2, 2);
changeIdx = find(dq > target_change_tol) + 1;

% Keep only changes after start
changeIdx = changeIdx(changeIdx > idx_global_start);

% Segment starts and ends
segStarts = [idx_global_start; changeIdx(:)];
segEnds   = [changeIdx(:)-1; N];

% Remove very short segments if any
minSamples = 5;
validSeg = (segEnds - segStarts + 1) >= minSamples;
segStarts = segStarts(validSeg);
segEnds   = segEnds(validSeg);

% Keep first 3 tasks
numTasks = min(3, numel(segStarts));

if numTasks < 3
    warning('Only %d task segment(s) detected. Check q_target switching.', numTasks);
end

fprintf('\nDetected task segments from q_target:\n');
for taskID = 1:numTasks
    i1 = segStarts(taskID);
    i2 = segEnds(taskID);

    fprintf('Task %d: idx %d to %d | t %.3f to %.3f | duration %.3f s | goal = [%.4f %.4f %.4f]\n', ...
        taskID, i1, i2, t_abs(i1), t_abs(i2), t_abs(i2)-t_abs(i1), ...
        q_target(i1,1), q_target(i1,2), q_target(i1,3));
end

%% --------- EXPORT EACH TASK ---------
Summary = table();

wrap = @(a) atan2(sin(a), cos(a));

for taskID = 1:numTasks

    idx_start = segStarts(taskID);
    idx_end   = segEnds(taskID);

    fprintf('\n============================================================\n');
    fprintf('Exporting Task %d\n', taskID);
    fprintf('idx_start = %d, t_start = %.3f s\n', idx_start, t_abs(idx_start));
    fprintf('idx_end   = %d, t_end   = %.3f s\n', idx_end, t_abs(idx_end));
    fprintf('duration  = %.3f s\n', t_abs(idx_end) - t_abs(idx_start));
    fprintf('============================================================\n');

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

    % Store absolute original interval too
    D_task.t_abs_start = t_abs(idx_start);
    D_task.t_abs_end   = t_abs(idx_end);
    D_task.idx_start_original = idx_start;
    D_task.idx_end_original   = idx_end;

    % Store task goal from q_target
    D_task.q_goal = D_task.q_target(1,:).';

    %% Print important signal sizes
    fprintf('\nImportant signals for Task %d:\n', taskID);

    importantNames = {
        'q_est'
        'Y_viscon'
        'Y_viscon_ground_trith'
        'q_est_imu'
        'q_target'
        'bot_accelerpmeter'
        'bot_gyroscope'
        'bot_wheelspeed'
        'v'
        'omega'
        'omega_wheels'
        't'
        'tout'
    };

    for j = 1:numel(importantNames)
        name = importantNames{j};

        if isfield(D_task,name)
            fprintf('%-35s size: ', name);
            disp(size(D_task.(name)));
        end
    end

    %% Choose reference signal
    [Yref, refName, hasRef] = chooseReference(D_task, referenceCandidates);

    q_goal = D_task.q_goal;

    duration = D_task.t(end) - D_task.t(1);

    finalRefPosErr = NaN;
    finalRefThetaErr = NaN;
    finalQestPosErr = NaN;
    finalQestThetaErr = NaN;
    meanQestRefXY = NaN;
    maxQestRefXY = NaN;
    meanImuRefXY = NaN;
    maxImuRefXY = NaN;
    maxCmdWheel = NaN;
    maxMeasuredWheel = NaN;

    %% q_est final error
    if isfield(D_task,'q_est')
        q_est = forceCols(D_task.q_est, 3);
        q_end = q_est(end,:);

        finalQestPosErr = norm(q_end(1:2).' - q_goal(1:2));
        finalQestThetaErr = abs(wrap(q_goal(3) - q_end(3)));

        fprintf('\nTask %d final q_est:\n', taskID);
        fprintf('[%.4f %.4f %.4f]\n', q_end(1), q_end(2), q_end(3));
        fprintf('q_est position error = %.4f m\n', finalQestPosErr);
        fprintf('q_est theta error    = %.4f rad = %.2f deg\n', ...
            finalQestThetaErr, rad2deg(finalQestThetaErr));
    end

    %% Reference final error and diagnostic comparisons
    if hasRef
        validRef = ~(abs(Yref(:,1)) < 1e-9 & abs(Yref(:,2)) < 1e-9 & abs(Yref(:,3)) < 1e-9);

        if any(validRef)
            Yv = Yref(validRef,:);
            Y_end = Yv(end,:);

            finalRefPosErr = norm(Y_end(1:2).' - q_goal(1:2));
            finalRefThetaErr = abs(wrap(q_goal(3) - Y_end(3)));

            fprintf('\nTask %d final reference (%s):\n', taskID, refName);
            fprintf('[%.4f %.4f %.4f]\n', Y_end(1), Y_end(2), Y_end(3));
            fprintf('Reference position error = %.4f m\n', finalRefPosErr);
            fprintf('Reference theta error    = %.4f rad = %.2f deg\n', ...
                finalRefThetaErr, rad2deg(finalRefThetaErr));

            if isfield(D_task,'q_est')
                q_est = forceCols(D_task.q_est, 3);
                M = min(size(q_est,1), size(Yref,1));
                q_est = q_est(1:M,:);
                Ytmp = Yref(1:M,:);
                validTmp = validRef(1:M);

                e_qest_xy = vecnorm(q_est(validTmp,1:2) - Ytmp(validTmp,1:2), 2, 2);

                meanQestRefXY = mean(e_qest_xy,'omitnan');
                maxQestRefXY = max(e_qest_xy,[],'omitnan');
            end

            if isfield(D_task,'q_est_imu')
                q_imu = forceCols(D_task.q_est_imu, 3);
                M = min(size(q_imu,1), size(Yref,1));
                q_imu = q_imu(1:M,:);
                Ytmp = Yref(1:M,:);
                validTmp = validRef(1:M);

                e_imu_xy = vecnorm(q_imu(validTmp,1:2) - Ytmp(validTmp,1:2), 2, 2);

                meanImuRefXY = mean(e_imu_xy,'omitnan');
                maxImuRefXY = max(e_imu_xy,[],'omitnan');
            end
        end
    end

    %% Wheel speed metrics
    if isfield(D_task,'omega_wheels')
        ow = D_task.omega_wheels;
        maxCmdWheel = max(abs(ow(:)),[],'omitnan');
    end

    if isfield(D_task,'bot_wheelspeed')
        bw = D_task.bot_wheelspeed;
        maxMeasuredWheel = max(abs(bw(:)),[],'omitnan');
    end

    %% Save task file
    saveName = sprintf('%s_task%d_all_outputs.mat', testPrefix, taskID);
    savePath = fullfile(scriptFolder, saveName);

    D_task1 = D_task; %#ok<NASGU>
    save(savePath, 'D_task1');

    fprintf('\nSaved Task %d file:\n%s\n', taskID, savePath);

    %% Add summary row
    Summary = [Summary; table( ...
        taskID, string(refName), duration, ...
        q_goal(1), q_goal(2), q_goal(3), ...
        finalQestPosErr, finalQestThetaErr, rad2deg(finalQestThetaErr), ...
        finalRefPosErr, finalRefThetaErr, rad2deg(finalRefThetaErr), ...
        meanQestRefXY, maxQestRefXY, ...
        meanImuRefXY, maxImuRefXY, ...
        maxCmdWheel, maxMeasuredWheel, ...
        t_abs(idx_start), t_abs(idx_end), ...
        'VariableNames', { ...
        'Task','ReferenceSignal','Duration_s', ...
        'Goal_x','Goal_y','Goal_theta', ...
        'Final_qest_pos_err_m','Final_qest_theta_err_rad','Final_qest_theta_err_deg', ...
        'Final_ref_pos_err_m','Final_ref_theta_err_rad','Final_ref_theta_err_deg', ...
        'Mean_qest_ref_XY_m','Max_qest_ref_XY_m', ...
        'Mean_imu_ref_XY_m','Max_imu_ref_XY_m', ...
        'Max_commanded_wheel_speed','Max_measured_wheel_speed', ...
        'Original_t_start_s','Original_t_end_s'} ...
    )];

end

%% --------- SAVE SUMMARY ---------
summaryPath = fullfile(scriptFolder, sprintf('%s_3tasks_summary.csv', testPrefix));
writetable(Summary, summaryPath);

fprintf('\n============================================================\n');
fprintf('Final 3-task summary:\n');
disp(Summary);

fprintf('\nSaved summary table:\n%s\n', summaryPath);

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
                X = forceCols(D.(name),3);
                Yref = X;
                refName = string(name);
                hasRef = true;
                return;
            catch
            end
        end
    end
end

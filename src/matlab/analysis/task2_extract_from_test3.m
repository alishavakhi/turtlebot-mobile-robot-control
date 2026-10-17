clear; clc; close all;

%% ============================================================
% Extract Task 2 from full real robot test
%
% Input:
%   realrobot_test3.mat
%
% Output:
%   test3_task2_full_outputs.mat
%
% Task_Manager logic:
%   Task 2 starts when q_target = q_wp1
%   Task 2 ends before q_target switches to q_wp3
% ============================================================

%% --------- FILE ---------
fullFile = 'realrobot_test3.mat';

if ~isfile(fullFile)
    error('Full real robot file not found: %s', fullFile);
end

S = load(fullFile);

fprintf('\nLoaded file: %s\n', fullFile);
fprintf('Variables inside file:\n');
disp(fieldnames(S));

%% --------- CONVERT FULL DATA TO STRUCT ---------
Dfull = struct();

% Case 1: file contains Simulink SimulationOutput variable out
if isfield(S, 'out')
    out = S.out;

    fprintf('\nDetected Simulink SimulationOutput object: out\n');

    outNames = out.who;
    fprintf('\nSignals inside out:\n');
    disp(outNames.');

    for i = 1:numel(outNames)
        name = outNames{i};

        try
            sig = out.get(name);
            Dfull.(name) = getRaw(sig);
        catch
            fprintf('Warning: could not extract signal: %s\n', name);
        end
    end

% Case 2: file already contains D_task2 / D_task1 / Dfull style struct
else
    vars = fieldnames(S);

    foundStruct = false;

    for i = 1:numel(vars)
        if isstruct(S.(vars{i})) && isfield(S.(vars{i}), 'q_target')
            Dfull = S.(vars{i});
            fprintf('\nDetected data structure: %s\n', vars{i});
            foundStruct = true;
            break;
        end
    end

    if ~foundStruct
        error('No SimulationOutput "out" and no structure with q_target found.');
    end
end

%% --------- REQUIRED SIGNAL CHECK ---------
if ~isfield(Dfull, 'q_target')
    error('q_target not found. Add To Workspace block for q_target in Simulink.');
end

q_target = forceCols(Dfull.q_target, 3);
Nfull = size(q_target, 1);

fprintf('\nFull data samples: %d\n', Nfull);

%% --------- TIME VECTOR ---------
if isfield(Dfull, 't')
    t_full = Dfull.t(:);
elseif isfield(Dfull, 'tout')
    t_full = Dfull.tout(:);
else
    t_full = (0:Nfull-1).' * 0.04;
    Dfull.t = t_full;
end

fprintf('Full duration = %.3f s\n', t_full(end) - t_full(1));

%% --------- TASK 2 TARGETS FROM YOUR TASK_MANAGER ---------
q_wp1   = [-0.20; -1.10; 0];
q_wp2   = [-0.20; -0.30; pi/2];
q_goal2 = [ 1.00; -0.30; 0];
q_wp3   = [ 1.45; -0.30; 0];

targetTol = 1e-3;

fprintf('\nUnique q_target rows in full test:\n');
uq = unique(round(q_target, 3), 'rows');
disp(uq);

%% --------- DETECT TASK 2 START ---------
is_wp1 = vecnorm(q_target(:,1:2) - q_wp1(1:2).', 2, 2) < targetTol;
idxStart = find(is_wp1, 1, 'first');

if isempty(idxStart)
    error('Task 2 start not found: q_wp1 never appears as q_target.');
end

fprintf('\nTask 2 start detected:\n');
fprintf('idxStart = %d\n', idxStart);
fprintf('tStart   = %.3f s\n', t_full(idxStart));

%% --------- DETECT TASK 2 END ---------
% Preferred end: just before Task_Manager switches to q_wp3.
is_wp3 = vecnorm(q_target(:,1:2) - q_wp3(1:2).', 2, 2) < targetTol;
idxWp3 = find(is_wp3 & ((1:Nfull).' > idxStart), 1, 'first');

if ~isempty(idxWp3)
    idxEnd = idxWp3 - 1;

    fprintf('\nSwitch to Task 3 detected:\n');
    fprintf('idxWp3 = %d\n', idxWp3);
    fprintf('Task 2 idxEnd = %d\n', idxEnd);

else
    % Fallback: last time q_target is q_goal2
    is_goal2 = vecnorm(q_target(:,1:2) - q_goal2(1:2).', 2, 2) < targetTol;
    idxGoal2Last = find(is_goal2 & ((1:Nfull).' > idxStart), 1, 'last');

    if isempty(idxGoal2Last)
        warning('q_wp3 and q_goal2 not found after Task 2 start. Exporting until end.');
        idxEnd = Nfull;
    else
        idxEnd = idxGoal2Last;
        fprintf('\nq_wp3 not found. Using last q_goal2 sample as Task 2 end.\n');
        fprintf('idxEnd = %d\n', idxEnd);
    end
end

if idxEnd <= idxStart
    error('Invalid Task 2 interval: idxEnd <= idxStart.');
end

fprintf('\nTask 2 interval:\n');
fprintf('idxStart = %d, idxEnd = %d\n', idxStart, idxEnd);
fprintf('tStart   = %.3f s\n', t_full(idxStart));
fprintf('tEnd     = %.3f s\n', t_full(idxEnd));
fprintf('duration = %.3f s\n', t_full(idxEnd) - t_full(idxStart));

%% --------- CROP ALL SIGNALS ---------
D_task2 = struct();

fields = fieldnames(Dfull);

for i = 1:numel(fields)
    f = fields{i};
    X = Dfull.(f);

    if isnumeric(X)
        Xs = squeeze(X);

        if size(Xs,1) == Nfull
            D_task2.(f) = Xs(idxStart:idxEnd, :);

        elseif size(Xs,2) == Nfull
            D_task2.(f) = Xs(:, idxStart:idxEnd);

        else
            D_task2.(f) = X;
        end
    else
        D_task2.(f) = X;
    end
end

%% --------- NORMALIZE TIME ---------
if isfield(D_task2, 't')
    D_task2.t = D_task2.t(:);
    D_task2.t = D_task2.t - D_task2.t(1);
elseif isfield(D_task2, 'tout')
    D_task2.tout = D_task2.tout(:);
    D_task2.tout = D_task2.tout - D_task2.tout(1);
else
    D_task2.t = (0:(idxEnd-idxStart)).' * 0.04;
end

%% --------- ADD METADATA ---------
D_task2.idx_start_original = idxStart;
D_task2.idx_end_original   = idxEnd;
D_task2.source_file        = fullFile;

D_task2.q_wp1   = q_wp1;
D_task2.q_wp2   = q_wp2;
D_task2.q_goal2 = q_goal2;
D_task2.q_wp3   = q_wp3;

%% --------- CHECK EXTRACTED DATA ---------
q_task2_target = forceCols(D_task2.q_target, 3);

fprintf('\nExtracted Task 2 unique q_target rows:\n');
disp(unique(round(q_task2_target, 3), 'rows'));

fprintf('\nExtracted first q_target:\n');
disp(q_task2_target(1,:).');

fprintf('Extracted final q_target:\n');
disp(q_task2_target(end,:).');

%% --------- FIND REAL POSE SIGNAL ---------
q_pose = [];
poseName = '';

if isfield(D_task2, 'Y_viscon_ground_trith')
    q_pose = forceCols(D_task2.Y_viscon_ground_trith, 3);
    poseName = 'Y_viscon_ground_trith';
elseif isfield(D_task2, 'Y_vicon_ground_truth')
    q_pose = forceCols(D_task2.Y_vicon_ground_truth, 3);
    poseName = 'Y_vicon_ground_truth';
elseif isfield(D_task2, 'Y_viscon')
    temp = D_task2.Y_viscon;
    if canBePose3(temp)
        q_pose = forceCols(temp, 3);
        poseName = 'Y_viscon';
    end
elseif isfield(D_task2, 'Y_vicon_used')
    temp = D_task2.Y_vicon_used;
    if canBePose3(temp)
        q_pose = forceCols(temp, 3);
        poseName = 'Y_vicon_used';
    end
elseif isfield(D_task2, 'q_est')
    q_pose = forceCols(D_task2.q_est, 3);
    poseName = 'q_est';
end

if ~isempty(q_pose)
    fprintf('\nPose signal used for check: %s\n', poseName);

    fprintf('First pose:\n');
    disp(q_pose(1,:).');

    fprintf('Final pose:\n');
    disp(q_pose(end,:).');

    pos_err_goal = norm(q_pose(end,1:2) - q_goal2(1:2).');
    fprintf('Final position error to Task 2 goal = %.4f m\n', pos_err_goal);
else
    warning('No valid pose signal found for final check.');
end

%% --------- QUICK XY PLOT ---------
figure('Color','w', 'Name','Extracted Task 2 check');
hold on; grid on; axis equal;

plot(q_task2_target(:,1), q_task2_target(:,2), 'k--', 'LineWidth', 1.4);

if ~isempty(q_pose)
    plot(q_pose(:,1), q_pose(:,2), 'b', 'LineWidth', 1.5);
    plot(q_pose(1,1), q_pose(1,2), 'go', 'MarkerFaceColor','g');
    plot(q_pose(end,1), q_pose(end,2), 'mo', 'MarkerFaceColor','m');
end

plot(q_wp1(1), q_wp1(2), 'ko', 'MarkerSize', 7, 'LineWidth', 1.4);
plot(q_wp2(1), q_wp2(2), 'ks', 'MarkerSize', 7, 'LineWidth', 1.4);
plot(q_goal2(1), q_goal2(2), 'kx', 'LineWidth', 2, 'MarkerSize', 10);

xlabel('x [m]');
ylabel('y [m]');
title('Extracted Task 2 from full real test');

legend('q target', 'real pose', 'start', 'end', ...
       'wp1', 'wp2', 'goal', ...
       'Location','bestoutside');

%% --------- SAVE ---------
saveName = 'test3_task2_full_outputs.mat';
save(saveName, 'D_task2');

fprintf('\nSaved extracted Task 2 data:\n%s\n', saveName);

%% ============================================================
% Helper functions
% ============================================================

function X = getRaw(sig)

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

function tf = canBePose3(X)

    X = squeeze(X);

    if isempty(X)
        tf = false;
        return;
    end

    tf = (size(X,2) == 3) || (size(X,1) == 3);
end

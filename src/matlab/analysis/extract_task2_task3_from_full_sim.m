clear; clc; close all;

%% ============================================================
% Extract Task 2 and Task 3 from full simulation data
%
% Input:
%   Full simulation .mat file
%
% Outputs:
%   exported_task2_simulation.mat
%   exported_task3_simulation.mat
%   exported_task2_simulation_from_full.mat
%   exported_task3_simulation_from_full.mat
%
% Notes:
%   - Task 2 starts when q_target = q_wp1
%   - Task 2 ends before q_target switches to q_wp3
%   - Task 3 starts when q_target = q_wp3
% ============================================================

%% ---------------- SCRIPT FOLDER ----------------
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    scriptFolder = pwd;
else
    scriptFolder = fileparts(scriptPath);
end

cd(scriptFolder);

fprintf('\nScript folder:\n%s\n', scriptFolder);

%% ---------------- USER SETTINGS ----------------
% Put your full simulation file name here:
fullSimFile = 'full_task_simulation.mat';

% If true, this overwrites exported_task2_simulation.mat so the plot script
% automatically uses the new extracted simulation data.
overwriteCurrentTask2Simulation = true;

Ts = 0.04;
r  = 0.03;
d  = 0.165;

q_goal_task1 = [-1.60; -1.10; 0];
q_goal_task2 = [ 1.00; -0.30; 0];

%% ---------------- WAYPOINTS FROM TASK MANAGER ----------------
q_wp1   = [-0.20; -1.10; 0];
q_wp2   = [-0.20; -0.30; pi/2];
q_goal2 = [ 1.00; -0.30; 0];

q_wp3   = [ 1.45; -0.30; 0];
q_start3 = [2.00; -0.60; 0];

targetTol = 1e-3;

%% ---------------- CHECK FILE ----------------
if ~isfile(fullSimFile)
    error('Full simulation file not found:\n%s', fullSimFile);
end

fprintf('\nLoading full simulation file:\n%s\n', fullSimFile);

S = load(fullSimFile);

fprintf('\nVariables inside file:\n');
disp(fieldnames(S));

%% ============================================================
% CONVERT FULL SIMULATION TO STANDARD STRUCTURE
% ============================================================

Dfull_raw = loadAnySimulationData(S);

Dfull = standardizeSimulationStruct(Dfull_raw, Ts, r, d);

q_target = forceCols(Dfull.q_target, 3);
q_actual = forceCols(Dfull.q_actual, 3);
q_est    = forceCols(Dfull.q_est, 3);
t        = Dfull.t(:);

N = min([size(q_target,1), size(q_actual,1), size(q_est,1), numel(t)]);

q_target = q_target(1:N,:);
q_actual = q_actual(1:N,:);
q_est    = q_est(1:N,:);
t        = t(1:N);

Dfull = cropStructToLength(Dfull, N);

fprintf('\nFull simulation samples: %d\n', N);
fprintf('Full simulation duration: %.3f s\n', t(end) - t(1));

fprintf('\nUnique q_target rows in full simulation:\n');
disp(unique(round(q_target,3), 'rows'));

%% ============================================================
% DETECT TASK 2 START
% ============================================================

is_wp1 = targetMatch(q_target, q_wp1, targetTol);
idxTask2Start = find(is_wp1, 1, 'first');

if isempty(idxTask2Start)
    error('Could not detect Task 2 start: q_wp1 never appears as q_target.');
end

fprintf('\nDetected Task 2 start:\n');
fprintf('idxTask2Start = %d\n', idxTask2Start);
fprintf('tTask2Start   = %.3f s\n', t(idxTask2Start));

%% ============================================================
% DETECT TASK 3 START
% ============================================================

is_wp3 = targetMatch(q_target, q_wp3, targetTol);
idxTask3Start = find(is_wp3 & ((1:N).' > idxTask2Start), 1, 'first');

if isempty(idxTask3Start)
    warning('q_wp3 was not found. Trying q_start3 as Task 3 start.');

    is_start3 = targetMatch(q_target, q_start3, targetTol);
    idxTask3Start = find(is_start3 & ((1:N).' > idxTask2Start), 1, 'first');
end

if isempty(idxTask3Start)
    warning('Could not detect Task 3 start from q_wp3 or q_start3.');

    is_goal2 = targetMatch(q_target, q_goal2, targetTol);
    idxGoal2Last = find(is_goal2 & ((1:N).' > idxTask2Start), 1, 'last');

    if isempty(idxGoal2Last)
        error('Could not detect Task 2 end either. Check q_target sequence.');
    end

    idxTask2End = idxGoal2Last;
    idxTask3Start = idxGoal2Last + 1;

    if idxTask3Start > N
        error('Task 3 start would be after the end of the file.');
    end
else
    idxTask2End = idxTask3Start - 1;
end

fprintf('\nDetected Task 2 end / Task 3 start:\n');
fprintf('idxTask2End   = %d\n', idxTask2End);
fprintf('tTask2End     = %.3f s\n', t(idxTask2End));
fprintf('idxTask3Start = %d\n', idxTask3Start);
fprintf('tTask3Start   = %.3f s\n', t(idxTask3Start));

%% ============================================================
% TASK 3 END
% ============================================================

idxTask3End = N;

fprintf('\nDetected Task 3 end:\n');
fprintf('idxTask3End = %d\n', idxTask3End);
fprintf('tTask3End   = %.3f s\n', t(idxTask3End));

%% ============================================================
% EXTRACT TASK 2 AND TASK 3
% ============================================================

Dsim_task2 = cropStructByIndex(Dfull, idxTask2Start:idxTask2End);
Dsim_task3 = cropStructByIndex(Dfull, idxTask3Start:idxTask3End);

% Normalize time
Dsim_task2.t = Dsim_task2.t(:);
Dsim_task2.t = Dsim_task2.t - Dsim_task2.t(1);

Dsim_task3.t = Dsim_task3.t(:);
Dsim_task3.t = Dsim_task3.t - Dsim_task3.t(1);

%% ---------------- ADD METADATA ----------------
Dsim_task2.source_file = fullSimFile;
Dsim_task2.idx_start_original = idxTask2Start;
Dsim_task2.idx_end_original   = idxTask2End;

Dsim_task2.q_goal_task1 = q_goal_task1;
Dsim_task2.q_goal_task2 = q_goal_task2;
Dsim_task2.q_wp1 = q_wp1;
Dsim_task2.q_wp2 = q_wp2;
Dsim_task2.q_goal2 = q_goal2;
Dsim_task2.Ts = Ts;
Dsim_task2.r = r;
Dsim_task2.d = d;

Dsim_task3.source_file = fullSimFile;
Dsim_task3.idx_start_original = idxTask3Start;
Dsim_task3.idx_end_original   = idxTask3End;

Dsim_task3.q_goal_task1 = q_goal_task1;
Dsim_task3.q_goal_task2 = q_goal_task2;
Dsim_task3.q_wp3 = q_wp3;
Dsim_task3.q_start3 = q_start3;
Dsim_task3.Ts = Ts;
Dsim_task3.r = r;
Dsim_task3.d = d;

%% ============================================================
% FINAL CHECKS
% ============================================================

fprintf('\n============================================================\n');
fprintf('TASK 2 EXTRACTED SIMULATION CHECK\n');
fprintf('============================================================\n');

q2_actual = forceCols(Dsim_task2.q_actual, 3);
q2_target = forceCols(Dsim_task2.q_target, 3);

fprintf('Task 2 duration = %.3f s\n', Dsim_task2.t(end));
fprintf('Task 2 first q_target:\n');
disp(q2_target(1,:).');

fprintf('Task 2 final q_target:\n');
disp(q2_target(end,:).');

fprintf('Task 2 unique q_target rows:\n');
disp(unique(round(q2_target,3),'rows'));

fprintf('Task 2 final q_actual:\n');
disp(q2_actual(end,:).');

task2_pos_err = norm(q2_actual(end,1:2) - q_goal2(1:2).');
task2_theta_err = abs(wrapToPiLocal(q_goal2(3) - q2_actual(end,3)));

fprintf('Task 2 final position error to goal = %.4f m\n', task2_pos_err);
fprintf('Task 2 final theta error to goal    = %.4f rad = %.2f deg\n', ...
    task2_theta_err, rad2deg(task2_theta_err));

fprintf('\n============================================================\n');
fprintf('TASK 3 EXTRACTED SIMULATION CHECK\n');
fprintf('============================================================\n');

q3_actual = forceCols(Dsim_task3.q_actual, 3);
q3_target = forceCols(Dsim_task3.q_target, 3);

fprintf('Task 3 duration = %.3f s\n', Dsim_task3.t(end));
fprintf('Task 3 first q_target:\n');
disp(q3_target(1,:).');

fprintf('Task 3 final q_target:\n');
disp(q3_target(end,:).');

fprintf('Task 3 unique first 10 q_target rows:\n');
uq3 = unique(round(q3_target,3),'rows');
disp(uq3(1:min(10,size(uq3,1)),:));

fprintf('Task 3 final q_actual:\n');
disp(q3_actual(end,:).');

%% ============================================================
% SAVE FILES
% ============================================================

save('exported_task2_simulation_from_full.mat', 'Dsim_task2');
save('exported_task3_simulation_from_full.mat', 'Dsim_task3');

fprintf('\nSaved:\n');
fprintf('  exported_task2_simulation_from_full.mat\n');
fprintf('  exported_task3_simulation_from_full.mat\n');

if overwriteCurrentTask2Simulation
    save('exported_task2_simulation.mat', 'Dsim_task2');
    fprintf('  exported_task2_simulation.mat  <-- overwritten for current plot script\n');
end

save('exported_task3_simulation.mat', 'Dsim_task3');
fprintf('  exported_task3_simulation.mat\n');

%% ============================================================
% QUICK PLOTS
% ============================================================

outDir = fullfile(scriptFolder, 'plots_task2');

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

fig1 = figure('Color','w', 'Name','Extracted Task 2 simulation check');
hold on; grid on; axis equal;

plot(q2_target(:,1), q2_target(:,2), 'k--', 'LineWidth', 1.3);
plot(q2_actual(:,1), q2_actual(:,2), 'g', 'LineWidth', 1.8);
plot(q2_actual(1,1), q2_actual(1,2), 'go', 'MarkerFaceColor','g');
plot(q2_actual(end,1), q2_actual(end,2), 'mo', 'MarkerFaceColor','m');
plot(q_wp1(1), q_wp1(2), 'ko', 'MarkerFaceColor','w');
plot(q_wp2(1), q_wp2(2), 'ks', 'MarkerFaceColor','w');
plot(q_goal2(1), q_goal2(2), 'kx', 'LineWidth',2, 'MarkerSize',10);

xlabel('x [m]');
ylabel('y [m]');
title('Extracted simulation - Task 2');
legend('q target','q actual','start','end','wp1','wp2','goal', ...
    'Location','bestoutside');

saveas(fig1, fullfile(outDir, 'check_extracted_task2_simulation.png'));

fig2 = figure('Color','w', 'Name','Extracted Task 3 simulation check');
hold on; grid on; axis equal;

plot(q3_target(:,1), q3_target(:,2), 'k--', 'LineWidth', 1.3);
plot(q3_actual(:,1), q3_actual(:,2), 'g', 'LineWidth', 1.8);
plot(q3_actual(1,1), q3_actual(1,2), 'go', 'MarkerFaceColor','g');
plot(q3_actual(end,1), q3_actual(end,2), 'mo', 'MarkerFaceColor','m');
plot(q_wp3(1), q_wp3(2), 'ko', 'MarkerFaceColor','w');
plot(q_start3(1), q_start3(2), 'ks', 'MarkerFaceColor','w');

xlabel('x [m]');
ylabel('y [m]');
title('Extracted simulation - Task 3');
legend('q target','q actual','start','end','wp3','circle start', ...
    'Location','bestoutside');

saveas(fig2, fullfile(outDir, 'check_extracted_task3_simulation.png'));

fprintf('\nSaved quick-check plots in:\n%s\n', outDir);

%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function D = loadAnySimulationData(S)

    D = struct();

    if isfield(S, 'out')
        out = S.out;

        if isa(out, 'Simulink.SimulationOutput')
            fprintf('\nDetected Simulink.SimulationOutput variable: out\n');

            names = out.who;
            fprintf('\nSignals inside out:\n');
            disp(names.');

            for i = 1:numel(names)
                name = names{i};

                try
                    D.(name) = getRaw(out.get(name));
                catch
                    fprintf('Warning: could not extract signal: %s\n', name);
                end
            end

            try
                D.tout = out.tout(:);
            catch
            end

            return;
        end
    end

    vars = fieldnames(S);

    for i = 1:numel(vars)
        if isstruct(S.(vars{i})) && isfield(S.(vars{i}), 'q_target')
            fprintf('\nDetected structure with q_target: %s\n', vars{i});
            D = S.(vars{i});
            return;
        end
    end

    % If variables are directly saved, use the loaded structure itself.
    if isfield(S, 'q_target')
        fprintf('\nDetected direct workspace variables.\n');
        D = S;
        return;
    end

    error('Could not find Simulink output or a structure containing q_target.');
end

function D = standardizeSimulationStruct(Din, Ts, r, d)

    D = struct();

    if isfield(Din, 'q_target')
        D.q_target = forceCols(Din.q_target, 3);
    else
        error('q_target not found in full simulation data.');
    end

    if isfield(Din, 'q_actual')
        D.q_actual = forceCols(Din.q_actual, 3);
    elseif isfield(Din, 'Y_vicon_ground_truth_sim')
        D.q_actual = forceCols(Din.Y_vicon_ground_truth_sim, 3);
    elseif isfield(Din, 'Y_viscon_ground_truth')
        [ok, X] = tryForceCols(Din.Y_viscon_ground_truth, 3);
        if ok
            D.q_actual = X;
        else
            D.q_actual = forceCols(Din.q_target, 3);
        end
    elseif isfield(Din, 'q_est')
        D.q_actual = forceCols(Din.q_est, 3);
    else
        error('No q_actual or equivalent pose signal found.');
    end

    if isfield(Din, 'q_est')
        D.q_est = forceCols(Din.q_est, 3);
    else
        D.q_est = D.q_actual;
    end

    if isfield(Din, 'v')
        D.v = forceCols(Din.v, 1);
    else
        D.v = zeros(size(D.q_actual,1),1);
    end

    if isfield(Din, 'omega')
        D.omega = forceCols(Din.omega, 1);
    else
        D.omega = zeros(size(D.q_actual,1),1);
    end

    if isfield(Din, 'omega_wheels')
        D.omega_wheels = forceCols(Din.omega_wheels, 2);
    else
        D.omega_wheels = wheelsFromVOmega(D.v, D.omega, r, d);
    end

    if isfield(Din, 'bot_wheelspeed')
        [ok, X] = tryForceCols(Din.bot_wheelspeed, 2);
        if ok
            D.bot_wheelspeed = X;
        else
            D.bot_wheelspeed = D.omega_wheels;
        end
    else
        D.bot_wheelspeed = D.omega_wheels;
    end

    if isfield(Din, 'q_est_imu')
        [ok, X] = tryForceCols(Din.q_est_imu, 3);
        if ok
            D.q_est_imu = X;
        else
            D.q_est_imu = D.q_est;
        end
    else
        D.q_est_imu = D.q_est;
    end

    if isfield(Din, 'u_actual')
        [ok, X] = tryForceCols(Din.u_actual, 2);
        if ok
            D.u_actual = X;
        else
            D.u_actual = [D.v, D.omega];
        end
    else
        D.u_actual = [D.v, D.omega];
    end

    if isfield(Din, 'Y_vicon_used')
        [ok, X] = tryForceCols(Din.Y_vicon_used, 3);
        if ok
            D.Y_vicon_used = X;
        else
            D.Y_vicon_used = D.q_actual;
        end
    elseif isfield(Din, 'Y_viscon')
        [ok, X] = tryForceCols(Din.Y_viscon, 3);
        if ok
            D.Y_vicon_used = X;
        else
            D.Y_vicon_used = D.q_actual;
        end
    else
        D.Y_vicon_used = D.q_actual;
    end

    if isfield(Din, 'Y_vicon_ground_truth_sim')
        [ok, X] = tryForceCols(Din.Y_vicon_ground_truth_sim, 3);
        if ok
            D.Y_vicon_ground_truth_sim = X;
        else
            D.Y_vicon_ground_truth_sim = D.q_actual;
        end
    else
        D.Y_vicon_ground_truth_sim = D.q_actual;
    end

    if isfield(Din, 't')
        D.t = forceCols(Din.t, 1);
    elseif isfield(Din, 'tout')
        D.t = forceCols(Din.tout, 1);
    else
        D.t = (0:size(D.q_target,1)-1).' * Ts;
    end

    D.t = D.t(:);
    D.t = D.t - D.t(1);
end

function D = cropStructToLength(D, N)

    fields = fieldnames(D);

    for i = 1:numel(fields)
        f = fields{i};
        X = D.(f);

        if isnumeric(X)
            Xs = squeeze(X);

            if size(Xs,1) >= N
                D.(f) = Xs(1:N,:);
            elseif size(Xs,2) >= N
                D.(f) = Xs(:,1:N);
            else
                D.(f) = X;
            end
        end
    end
end

function Dcrop = cropStructByIndex(D, idx)

    Dcrop = struct();
    fields = fieldnames(D);
    N = size(D.q_target,1);

    for i = 1:numel(fields)
        f = fields{i};
        X = D.(f);

        if isnumeric(X)
            Xs = squeeze(X);

            if size(Xs,1) == N
                Dcrop.(f) = Xs(idx,:);
            elseif size(Xs,2) == N
                Dcrop.(f) = Xs(:,idx);
            else
                Dcrop.(f) = X;
            end
        else
            Dcrop.(f) = X;
        end
    end
end

function tf = targetMatch(q_target, q_ref, tol)

    tf = vecnorm(q_target(:,1:2) - q_ref(1:2).', 2, 2) < tol;
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
        X = X(:);
    end

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Expected %d columns, got size %dx%d.', ncols, size(X,1), size(X,2));
    end
end

function [ok, X] = tryForceCols(raw, ncols)

    ok = false;
    X = [];

    try
        Xtmp = forceCols(raw, ncols);
        ok = true;
        X = Xtmp;
    catch
        ok = false;
        X = [];
    end
end

function a = wrapToPiLocal(a)

    a = atan2(sin(a), cos(a));
end

function omega_wheels = wheelsFromVOmega(v, omega, r, d)

    v = v(:);
    omega = omega(:);

    omega_L = (v - omega * d/2) / r;
    omega_R = (v + omega * d/2) / r;

    omega_wheels = [omega_L, omega_R];
end

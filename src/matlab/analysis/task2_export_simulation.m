clear; clc; close all;

%% ============================================================
% Robust export script for Task 2 simulation
%
% Required files in the same folder:
%   task2_simulation.slx
%   test3_task2_full_outputs.mat
%
% Output:
%   exported_task2_simulation.mat
%
% This script:
%   1. Runs task2_simulation.slx from the real Task 2 initial pose.
%   2. Extracts the useful Task 2 interval.
%   3. Saves a clean Dsim_task2 structure for the plotting script.
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

%% ---------------- FILES ----------------
modelName = 'task2_simulation';
modelFile = fullfile(scriptFolder, [modelName '.slx']);
realFile  = fullfile(scriptFolder, 'test3_task2_full_outputs.mat');
saveFile  = fullfile(scriptFolder, 'exported_task2_simulation.mat');

if ~isfile(modelFile)
    error('Simulink model not found:\n%s', modelFile);
end

if ~isfile(realFile)
    error('Real data file not found:\n%s', realFile);
end

%% ---------------- PARAMETERS ----------------
% Mode
is_real_robot = 0;

% Sampling and robot parameters
Ts = 0.04;
r  = 0.03;
d  = 0.165;

% Controller gains
k_x     = 0.25;
k_y     = 1.2;
k_theta = 1;

% Saturations
% Use larger omega_max in simulation so the simulated robot can turn
% reasonably close to the real robot response.
v_max     = 0.22;
omega_max = 1.8;
omega_M   = 10;

% Goals
q_goal_task1 = [-1.60; -1.10; 0];
q_goal_task2 = [ 1.00; -0.30; 0];

% Task 2 waypoints
q_wp1   = [-0.20; -1.10; 0];
q_wp2   = [-0.20; -0.30; pi/2];
q_goal2 = q_goal_task2;

% Possible Task 3 targets.
% These are used only to crop Task 2 if the simulation continues.
q_wp3     = [1.45; -0.30; 0];
q_start3  = [2.00; -0.60; 0];

% Localization/noise parameter
sigma_e = 0.02;

% Simulation stop time
simStopTime = 35;

% Cropping settings
dist_threshold = 0.05;
marginTime = 1.0;
targetTol = 1e-3;

%% ---------------- GET INITIAL POSE FROM REAL DATA ----------------
Sreal = load(realFile);

if isfield(Sreal, 'D_task2')
    Dreal = Sreal.D_task2;
else
    error('D_task2 not found inside:\n%s', realFile);
end

q_real_start_all = getPoseFromRealStruct(Dreal);

q_init = q_real_start_all(1,:).';
q_init(3) = wrapToPiLocal(q_init(3));

fprintf('\nq_init used for simulation:\n');
disp(q_init);

fprintf('q_goal_task1:\n');
disp(q_goal_task1);

fprintf('q_goal_task2:\n');
disp(q_goal_task2);

fprintf('Controller and limits:\n');
fprintf('k_x = %.3f, k_y = %.3f, k_theta = %.3f\n', k_x, k_y, k_theta);
fprintf('v_max = %.3f m/s, omega_max = %.3f rad/s, omega_M = %.3f rad/s\n', ...
    v_max, omega_max, omega_M);

fprintf('\nSimulation stop time = %.3f s\n', simStopTime);

%% ---------------- PUSH VARIABLES TO BASE WORKSPACE ----------------
% This makes the script robust even if Simulink resolves variables from
% the base workspace.
assignin('base', 'is_real_robot', is_real_robot);
assignin('base', 'Ts', Ts);
assignin('base', 'r', r);
assignin('base', 'd', d);

assignin('base', 'k_x', k_x);
assignin('base', 'k_y', k_y);
assignin('base', 'k_theta', k_theta);

assignin('base', 'v_max', v_max);
assignin('base', 'omega_max', omega_max);
assignin('base', 'omega_M', omega_M);

assignin('base', 'q_goal_task1', q_goal_task1);
assignin('base', 'q_goal_task2', q_goal_task2);
assignin('base', 'q_goal2', q_goal2);

assignin('base', 'q_wp1', q_wp1);
assignin('base', 'q_wp2', q_wp2);
assignin('base', 'q_wp3', q_wp3);
assignin('base', 'q_start3', q_start3);

assignin('base', 'q_init', q_init);
assignin('base', 'sigma_e', sigma_e);

%% ---------------- RUN SIMULATION ----------------
fprintf('\nRunning model: %s\n', modelName);

load_system(modelName);

try
    set_param(modelName, 'FastRestart', 'off');
catch
end

try
    set_param(modelName, 'SimulationMode', 'normal');
catch
end

set_param(modelName, 'StopTime', num2str(simStopTime));

% Clear old SimulationOutput variable if it exists in base workspace
evalin('base', 'clear out');

out = sim(modelName, ...
    'ReturnWorkspaceOutputs', 'on');

fprintf('\nSimulation finished.\n');

%% ---------------- DISPLAY AVAILABLE OUTPUTS ----------------
try
    outNames = out.who;
    fprintf('\nSimulation output signals:\n');
    disp(outNames.');
catch
    warning('Could not display simulation output names.');
end

%% ---------------- EXTRACT REQUIRED SIGNALS ----------------
q_est    = getRequiredPose(out, {'q_est'});
q_actual = getRequiredPose(out, {'q_actual'});
q_target = getRequiredPose(out, {'q_target'});

v            = getRequiredScalar(out, {'v'});
omega        = getRequiredScalar(out, {'omega'});
omega_wheels = getRequiredMatrix(out, {'omega_wheels'}, 2);

Nbase = size(q_actual,1);

%% ---------------- OPTIONAL SIGNALS ----------------
q_est_imu = getOptionalMatrix(out, {'q_est_imu'}, 3, Nbase);
u_actual  = getOptionalMatrix(out, {'u_actual'}, 2, Nbase);

Y_vicon_used = getOptionalPoseWithFallback( ...
    out, {'Y_vicon_used', 'Y_viscon_used', 'Y_viscon'}, q_actual);

Y_vicon_ground_truth_sim = getOptionalPoseWithFallback( ...
    out, {'Y_vicon_ground_truth_sim', 'Y_viscon_ground_truth', ...
          'Y_viscon_ground_trith'}, q_actual);

%% ---------------- TIME ----------------
t = getTimeVector(out, size(q_actual,1), Ts);

%% ---------------- ALIGN LENGTHS ----------------
N = min([
    size(q_est,1)
    size(q_actual,1)
    size(q_target,1)
    size(v,1)
    size(omega,1)
    size(omega_wheels,1)
    size(q_est_imu,1)
    size(u_actual,1)
    size(Y_vicon_used,1)
    size(Y_vicon_ground_truth_sim,1)
    size(t,1)
]);

q_est = q_est(1:N,:);
q_actual = q_actual(1:N,:);
q_target = q_target(1:N,:);
v = v(1:N,:);
omega = omega(1:N,:);
omega_wheels = omega_wheels(1:N,:);
q_est_imu = q_est_imu(1:N,:);
u_actual = u_actual(1:N,:);
Y_vicon_used = Y_vicon_used(1:N,:);
Y_vicon_ground_truth_sim = Y_vicon_ground_truth_sim(1:N,:);
t = t(1:N,:);

t = t - t(1);

% Wrap all angles
q_est(:,3) = wrapToPiLocal(q_est(:,3));
q_actual(:,3) = wrapToPiLocal(q_actual(:,3));
q_target(:,3) = wrapToPiLocal(q_target(:,3));
q_est_imu(:,3) = wrapToPiLocal(q_est_imu(:,3));
Y_vicon_used(:,3) = wrapToPiLocal(Y_vicon_used(:,3));
Y_vicon_ground_truth_sim(:,3) = wrapToPiLocal(Y_vicon_ground_truth_sim(:,3));

fprintf('\nFirst q_target from simulation:\n');
disp(q_target(1,:).');

fprintf('Middle q_target from simulation:\n');
disp(q_target(round(N/2),:).');

fprintf('Final q_target before cropping:\n');
disp(q_target(end,:).');

fprintf('\nUnique q_target rows before cropping:\n');
disp(unique(round(q_target,3),'rows'));

%% ============================================================
% ROBUST TASK 2 CROPPING
% ============================================================

% Task 2 normally starts from the beginning of this model.
% But if the model produces initial transients or previous targets, start
% at the first q_wp1 occurrence when available.
is_wp1 = targetMatch(q_target, q_wp1, targetTol);
idxStart = find(is_wp1, 1, 'first');

if isempty(idxStart)
    idxStart = 1;
    fprintf('\nq_wp1 not detected as q_target. Using idxStart = 1.\n');
else
    fprintf('\nDetected q_wp1 as Task 2 start:\n');
    fprintf('idxStart = %d, t = %.3f s\n', idxStart, t(idxStart));
end

% End mode 1: if the simulation continues to Task 3, stop before Task 3.
is_wp3 = targetMatch(q_target, q_wp3, 0.05);
is_start3 = targetMatch(q_target, q_start3, 0.05);

idxTask3 = find((is_wp3 | is_start3) & ((1:N).' > idxStart), 1, 'first');

if ~isempty(idxTask3)
    idxEnd = max(idxStart, idxTask3 - 1);
    reachedFlag = true;

    fprintf('\nDetected next-task target. Cropping before Task 3:\n');
    fprintf('idxTask3 = %d, t = %.3f s\n', idxTask3, t(idxTask3));

else
    % End mode 2: crop when final goal is reached, plus marginTime.
    pos_err_to_goal = vecnorm(q_actual(:,1:2) - q_goal2(1:2).', 2, 2);

    idxReached = find(pos_err_to_goal < dist_threshold & ...
                      ((1:N).' >= idxStart) & t > 0.5, 1, 'first');

    if ~isempty(idxReached)
        reachedFlag = true;
        tReach = t(idxReached);
        idxEnd = find(t <= tReach + marginTime, 1, 'last');

        if isempty(idxEnd)
            idxEnd = idxReached;
        end

        fprintf('\nSimulation reached final position threshold:\n');
        fprintf('idxReached = %d\n', idxReached);
        fprintf('t = %.3f s\n', tReach);
        fprintf('position error = %.4f m\n', pos_err_to_goal(idxReached));

    else
        % End mode 3: use the last sample where target is still goal2.
        is_goal2 = targetMatch(q_target, q_goal2, 0.05);
        idxLastGoalTarget = find(is_goal2 & ((1:N).' >= idxStart), 1, 'last');

        if ~isempty(idxLastGoalTarget)
            idxEnd = idxLastGoalTarget;
            reachedFlag = false;

            warning('Final threshold not reached. Cropping at last q_goal2 target sample.');
        else
            idxEnd = N;
            reachedFlag = false;

            warning('Could not detect final goal reach or target transition. Exporting until the end.');
        end
    end
end

if idxEnd <= idxStart
    warning('Invalid crop indices detected. Falling back to full simulation.');
    idxStart = 1;
    idxEnd = N;
end

idx = idxStart:idxEnd;

fprintf('\nCropping summary:\n');
fprintf('idxStart = %d, tStart = %.3f s\n', idxStart, t(idxStart));
fprintf('idxEnd   = %d, tEnd   = %.3f s\n', idxEnd, t(idxEnd));
fprintf('Exported samples = %d\n', numel(idx));

%% ---------------- BUILD EXPORT STRUCT ----------------
Dsim_task2 = struct();

Dsim_task2.q_est = q_est(idx,:);
Dsim_task2.q_actual = q_actual(idx,:);
Dsim_task2.q_target = q_target(idx,:);

Dsim_task2.v = v(idx,:);
Dsim_task2.omega = omega(idx,:);
Dsim_task2.omega_wheels = omega_wheels(idx,:);

Dsim_task2.q_est_imu = q_est_imu(idx,:);
Dsim_task2.u_actual = u_actual(idx,:);

% No measured wheel-speed signal exists in simulation.
% For compatibility with plotting scripts, use commanded wheel speeds.
Dsim_task2.bot_wheelspeed = omega_wheels(idx,:);

Dsim_task2.Y_vicon_used = Y_vicon_used(idx,:);
Dsim_task2.Y_vicon_ground_truth_sim = Y_vicon_ground_truth_sim(idx,:);

Dsim_task2.t = t(idx,:);
Dsim_task2.t = Dsim_task2.t(:);
Dsim_task2.t = Dsim_task2.t - Dsim_task2.t(1);

Dsim_task2.q_init = q_init;
Dsim_task2.q_goal_task1 = q_goal_task1;
Dsim_task2.q_goal_task2 = q_goal_task2;

Dsim_task2.q_wp1 = q_wp1;
Dsim_task2.q_wp2 = q_wp2;
Dsim_task2.q_goal2 = q_goal2;

Dsim_task2.Ts = Ts;
Dsim_task2.r = r;
Dsim_task2.d = d;
Dsim_task2.k_x = k_x;
Dsim_task2.k_y = k_y;
Dsim_task2.k_theta = k_theta;
Dsim_task2.v_max = v_max;
Dsim_task2.omega_max = omega_max;
Dsim_task2.omega_M = omega_M;
Dsim_task2.sigma_e = sigma_e;

Dsim_task2.reachedFlag = reachedFlag;
Dsim_task2.idx_start_original = idxStart;
Dsim_task2.idx_end_original = idxEnd;
Dsim_task2.source_model = modelName;

%% ---------------- FINAL CHECKS ----------------
q_end = Dsim_task2.q_actual(end,:);
q_target_end = Dsim_task2.q_target(end,:);

pos_err_final = norm(q_end(1:2) - q_goal2(1:2).');
theta_err_final = abs(wrapToPiLocal(q_goal2(3) - q_end(3)));

fprintf('\n============================================================\n');
fprintf('EXPORTED TASK 2 SIMULATION CHECK\n');
fprintf('============================================================\n');

fprintf('Exported duration = %.3f s\n', Dsim_task2.t(end));

fprintf('\nFinal exported q_target:\n');
fprintf('[%.4f %.4f %.4f]\n', q_target_end(1), q_target_end(2), q_target_end(3));

fprintf('\nFinal exported simulation q_actual:\n');
fprintf('[%.4f %.4f %.4f]\n', q_end(1), q_end(2), q_end(3));

fprintf('Simulation position error to final goal = %.4f m\n', pos_err_final);
fprintf('Simulation theta error to final goal    = %.4f rad = %.2f deg\n', ...
        theta_err_final, rad2deg(theta_err_final));

fprintf('\nUnique exported q_target rows:\n');
disp(unique(round(Dsim_task2.q_target,3),'rows'));

%% ---------------- SAVE ----------------
save(saveFile, 'Dsim_task2');

fprintf('\nSaved simulation data here:\n%s\n', saveFile);

fprintf('\nSaved fields in Dsim_task2:\n');
disp(fieldnames(Dsim_task2));

%% ---------------- QUICK CHECK PLOT ----------------
fig = figure('Color','w', 'Name','Task 2 simulation export check');
hold on; grid on; axis equal;

plot(Dsim_task2.q_target(:,1), Dsim_task2.q_target(:,2), ...
    'k--', 'LineWidth', 1.3);

plot(Dsim_task2.q_actual(:,1), Dsim_task2.q_actual(:,2), ...
    'g', 'LineWidth', 2.0);

plot(Dsim_task2.q_actual(1,1), Dsim_task2.q_actual(1,2), ...
    'go', 'MarkerFaceColor','g', 'MarkerSize',8);

plot(Dsim_task2.q_actual(end,1), Dsim_task2.q_actual(end,2), ...
    'mo', 'MarkerFaceColor','m', 'MarkerSize',8);

plot(q_wp1(1), q_wp1(2), 'ko', ...
    'MarkerFaceColor','w', 'MarkerSize',8);

plot(q_wp2(1), q_wp2(2), 'ks', ...
    'MarkerFaceColor','w', 'MarkerSize',8);

plot(q_goal2(1), q_goal2(2), 'kx', ...
    'MarkerSize',12, 'LineWidth',2);

xlabel('x [m]');
ylabel('y [m]');
title('Exported Task 2 simulation check');
legend('q target','q actual','start','end','wp1','wp2','goal', ...
    'Location','bestoutside');

checkFolder = fullfile(scriptFolder, 'plots_task2');

if ~exist(checkFolder, 'dir')
    mkdir(checkFolder);
end

exportgraphics(fig, fullfile(checkFolder, 'check_exported_task2_simulation.png'), ...
    'Resolution',300, 'BackgroundColor','white');

fprintf('\nSaved quick-check plot:\n%s\n', ...
    fullfile(checkFolder, 'check_exported_task2_simulation.png'));

%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function q = getPoseFromRealStruct(Dreal)

    if isfield(Dreal, 'Y_viscon_ground_trith')
        q = forceCols(Dreal.Y_viscon_ground_trith, 3);
    elseif isfield(Dreal, 'Y_vicon_ground_truth')
        q = forceCols(Dreal.Y_vicon_ground_truth, 3);
    elseif isfield(Dreal, 'Y_viscon')
        q = forceCols(Dreal.Y_viscon, 3);
    elseif isfield(Dreal, 'q_est')
        q = forceCols(Dreal.q_est, 3);
    else
        error('No suitable pose signal found in real data.');
    end

    q(:,3) = wrapToPiLocal(q(:,3));
end

function q = getRequiredPose(out, names)

    q = getRequiredMatrix(out, names, 3);
    q(:,3) = wrapToPiLocal(q(:,3));
end

function x = getRequiredScalar(out, names)

    x = getRequiredMatrix(out, names, 1);
end

function X = getRequiredMatrix(out, names, ncols)

    for i = 1:numel(names)
        name = names{i};

        if hasOut(out, name)
            raw = getRaw(getOut(out, name));
            [ok, X] = tryForceCols(raw, ncols);

            if ok
                return;
            else
                fprintf('Signal "%s" exists but is not %d-column.\n', name, ncols);
            end
        end
    end

    error('None of the required signals were found with %d columns: %s', ...
        ncols, strjoin(names, ', '));
end

function X = getOptionalMatrix(out, names, ncols, Ndefault)

    for i = 1:numel(names)
        name = names{i};

        if hasOut(out, name)
            raw = getRaw(getOut(out, name));
            [ok, X] = tryForceCols(raw, ncols);

            if ok
                return;
            else
                fprintf('Optional signal "%s" exists but is not %d-column.\n', ...
                    name, ncols);
            end
        end
    end

    X = NaN(Ndefault, ncols);
    fprintf('Optional signal not found. Filled with NaN. Names tried: %s\n', ...
        strjoin(names, ', '));
end

function Q = getOptionalPoseWithFallback(out, names, fallback)

    Ndefault = size(fallback,1);

    Q = getOptionalMatrix(out, names, 3, Ndefault);

    if all(isnan(Q(:)))
        Q = fallback;
    end

    Q(:,3) = wrapToPiLocal(Q(:,3));
end

function t = getTimeVector(out, Ndefault, Ts)

    if hasOut(out, 't')
        raw = getRaw(getOut(out, 't'));
        [ok, t] = tryForceCols(raw, 1);

        if ok
            t = t(:);
            return;
        end
    end

    if hasOut(out, 'tout')
        raw = getRaw(getOut(out, 'tout'));
        [ok, t] = tryForceCols(raw, 1);

        if ok
            t = t(:);
            return;
        end
    end

    try
        t = out.tout(:);
    catch
        t = (0:Ndefault-1).' * Ts;
    end
end

function tf = hasOut(out, name)

    tf = false;

    try
        tf = any(strcmp(out.who, name));
    catch
        tf = false;
    end
end

function sig = getOut(out, name)

    if hasOut(out, name)
        sig = out.get(name);
    else
        error('Signal "%s" not found in simulation output.', name);
    end
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
        error('Expected %d columns, got size %dx%d.', ...
            ncols, size(X,1), size(X,2));
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

function tf = targetMatch(q_target, q_ref, tol)

    q_target = forceCols(q_target, 3);

    posMatch = vecnorm(q_target(:,1:2) - q_ref(1:2).', 2, 2) < tol;

    % Only use x-y matching. Theta may differ slightly because of wrapping.
    tf = posMatch;
end

function a = wrapToPiLocal(a)

    a = atan2(sin(a), cos(a));
end
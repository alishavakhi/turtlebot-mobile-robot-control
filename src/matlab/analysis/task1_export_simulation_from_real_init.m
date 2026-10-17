clear; clc; close all;

%% ============================================================
% Run Task 1 simulation from real Test 3 initial condition
% EXTENDED VERSION:
% The simulation is allowed to run longer so that orientation can converge.
%
% Required files in the same folder:
%   test3_task1_all_outputs.mat
%   task1_simulation.slx
%
% Output:
%   exported_task1_simulation_extended_test3.mat
% ============================================================

%% --------- FIND SCRIPT FOLDER ---------
scriptPath = mfilename('fullpath');

if isempty(scriptPath)
    scriptFolder = pwd;
else
    scriptFolder = fileparts(scriptPath);
end

fprintf('\nScript folder:\n%s\n', scriptFolder);

%% --------- FILES ---------
realFile = fullfile(scriptFolder, 'test3_task1_all_outputs.mat');
modelName = 'task1_simulation';
modelFile = fullfile(scriptFolder, [modelName '.slx']);

if ~isfile(realFile)
    error('Real data file not found:\n%s', realFile);
end

if ~isfile(modelFile)
    error('Simulink model not found:\n%s', modelFile);
end

%% --------- LOAD REAL TASK 1 DATA ---------
S = load(realFile);

if isfield(S,'D_task1')
    Dreal = S.D_task1;
else
    error('D_task1 not found inside real data file.');
end

%% --------- USE REAL REFERENCE AS INITIAL CONDITION ---------
if isfield(Dreal, 'Y_viscon_ground_trith')
    q_ref = forceCols(Dreal.Y_viscon_ground_trith, 3);
elseif isfield(Dreal, 'Y_viscon')
    q_ref = forceCols(Dreal.Y_viscon, 3);
else
    error('No Vicon/reference pose found in real data.');
end

q_target_real = forceCols(Dreal.q_target, 3);

% Initial condition from real Task 1
q_init = q_ref(1,:).';

% Goal from real Task 1 target
q_goal_task1 = q_target_real(1,:).';

fprintf('\nq_init used for simulation:\n');
disp(q_init);

fprintf('q_goal_task1 used for simulation:\n');
disp(q_goal_task1);

%% --------- SIMULATION PARAMETERS ---------
% These variables must match the names used inside Simulink.

is_real_robot = 0;      % simulation mode

Ts = 0.04;
r  = 0.03;
d  = 0.165;

dist_threshold = 0.05;

% Controller gains
% Keep the same values used in your current setup.
k_x = 0.5;
k_y = 1.2;
k_theta = 0.3;

% Limits
v_max = 0.22;
omega_max = 1.3;

% For Task 1-only simulation, keep the next goal equal to Task 1 goal.
% This avoids switching to another target.
q_goal_task2 = q_goal_task1;

% Map coordinates if your model needs them
x_coords = [-2.90, -1.60, -1.50, -1.40, -0.60, 0.20, 1.00, 1.45, 1.80, 2.20, 2.90];
y_coords = [-1.50, -1.10, -0.70, -0.30, -0.20, 0.10, 0.20, 1.50];

%% --------- EXTENDED SIMULATION TIME ---------
% Real Task 1 ended around 12.44 s, but orientation did not converge.
% Here we run longer to check if the same controller can reduce theta error.
realDuration = Dreal.t(end);

simStopTime = 40.0;

fprintf('\nReal Task 1 duration = %.3f s\n', realDuration);
fprintf('Extended simulation stop time = %.3f s\n', simStopTime);

%% --------- RUN SIMULINK ---------
open_system(modelName);
set_param(modelName, 'StopTime', num2str(simStopTime));

fprintf('\nRunning extended Simulink model...\n');
out = sim(modelName);
fprintf('\nSimulation finished.\n');

fprintf('\nSimulation output signals:\n');
disp(out.who);

%% --------- CHECK REQUIRED OUTPUTS ---------
requiredFields = {'q_est','q_actual','q_target','v','omega','omega_wheels'};

for i = 1:numel(requiredFields)
    if ~isprop(out, requiredFields{i})
        error('Missing simulation output "%s". Add a To Workspace block for this signal in Simulink.', requiredFields{i});
    end
end

%% --------- EXPORT SIMULATION DATA ---------
Dsim_task1_simulation = struct();

Dsim_task1_simulation.q_est        = forceCols(getRaw(out.q_est), 3);
Dsim_task1_simulation.q_actual     = forceCols(getRaw(out.q_actual), 3);
Dsim_task1_simulation.q_target     = forceCols(getRaw(out.q_target), 3);
Dsim_task1_simulation.v            = getRaw(out.v);
Dsim_task1_simulation.omega        = getRaw(out.omega);
Dsim_task1_simulation.omega_wheels = forceCols(getRaw(out.omega_wheels), 2);

Dsim_task1_simulation.v = Dsim_task1_simulation.v(:);
Dsim_task1_simulation.omega = Dsim_task1_simulation.omega(:);

if isprop(out, 'q_est_imu')
    Dsim_task1_simulation.q_est_imu = forceCols(getRaw(out.q_est_imu), 3);
else
    Dsim_task1_simulation.q_est_imu = NaN(size(Dsim_task1_simulation.q_est));
end

if isprop(out, 'u_actual')
    Dsim_task1_simulation.u_actual = forceCols(getRaw(out.u_actual), 2);
else
    Dsim_task1_simulation.u_actual = NaN(length(Dsim_task1_simulation.v), 2);
end

if isprop(out, 'tout')
    Dsim_task1_simulation.t = out.tout(:);
elseif isprop(out, 't')
    Dsim_task1_simulation.t = getRaw(out.t);
    Dsim_task1_simulation.t = Dsim_task1_simulation.t(:);
else
    Dsim_task1_simulation.t = (0:size(Dsim_task1_simulation.q_actual,1)-1).' * Ts;
end

%% --------- MAKE ALL SIGNALS SAME LENGTH ---------
N = min([
    size(Dsim_task1_simulation.q_est,1)
    size(Dsim_task1_simulation.q_actual,1)
    size(Dsim_task1_simulation.q_target,1)
    size(Dsim_task1_simulation.q_est_imu,1)
    size(Dsim_task1_simulation.v,1)
    size(Dsim_task1_simulation.omega,1)
    size(Dsim_task1_simulation.omega_wheels,1)
    size(Dsim_task1_simulation.u_actual,1)
    size(Dsim_task1_simulation.t,1)
]);

fields = fieldnames(Dsim_task1_simulation);

for i = 1:numel(fields)
    f = fields{i};
    Dsim_task1_simulation.(f) = Dsim_task1_simulation.(f)(1:N,:);
end

Dsim_task1_simulation.t = Dsim_task1_simulation.t - Dsim_task1_simulation.t(1);

%% --------- FIND CONVERGENCE TIME ---------
wrap = @(a) atan2(sin(a), cos(a));

q_actual = Dsim_task1_simulation.q_actual;
t_sim = Dsim_task1_simulation.t;

e_pos = vecnorm(q_actual(:,1:2) - q_goal_task1(1:2).', 2, 2);
e_th  = abs(wrap(q_goal_task1(3) - q_actual(:,3)));

posTol = 0.05;
thetaTol = 0.05;

idxConv = find(e_pos <= posTol & e_th <= thetaTol, 1, 'first');

if ~isempty(idxConv)
    fprintf('\nSimulation reached both thresholds:\n');
    fprintf('t = %.3f s\n', t_sim(idxConv));
    fprintf('position error = %.4f m\n', e_pos(idxConv));
    fprintf('theta error    = %.4f rad = %.2f deg\n', e_th(idxConv), rad2deg(e_th(idxConv)));

    % Keep a small margin after convergence for plotting.
    marginTime = 1.0;
    idxEnd = min(length(t_sim), idxConv + round(marginTime/Ts));

else
    fprintf('\nSimulation did NOT reach both thresholds within %.2f s.\n', simStopTime);
    fprintf('Keeping full extended simulation.\n');

    idxEnd = length(t_sim);
end

%% --------- CROP TO CONVERGENCE + MARGIN IF AVAILABLE ---------
fields = fieldnames(Dsim_task1_simulation);

for i = 1:numel(fields)
    f = fields{i};

    if isnumeric(Dsim_task1_simulation.(f)) && size(Dsim_task1_simulation.(f),1) >= idxEnd
        Dsim_task1_simulation.(f) = Dsim_task1_simulation.(f)(1:idxEnd,:);
    end
end

%% --------- FINAL SIMULATION ERROR AFTER CROPPING ---------
q_end = Dsim_task1_simulation.q_actual(end,:);

pos_err = norm(q_end(1:2).' - q_goal_task1(1:2));
theta_err = abs(wrap(q_goal_task1(3) - q_end(3)));

fprintf('\nFinal exported simulation q_actual:\n');
fprintf('[%.4f %.4f %.4f]\n', q_end(1), q_end(2), q_end(3));
fprintf('Simulation position error = %.4f m\n', pos_err);
fprintf('Simulation theta error    = %.4f rad = %.2f deg\n', theta_err, rad2deg(theta_err));
fprintf('Exported simulation duration = %.3f s\n', Dsim_task1_simulation.t(end));

%% --------- SAVE ---------
savePath = fullfile(scriptFolder, 'exported_task1_simulation_extended_test3.mat');
save(savePath, 'Dsim_task1_simulation');

fprintf('\nSaved extended simulation data here:\n%s\n', savePath);

fprintf('\nSaved fields:\n');
disp(fieldnames(Dsim_task1_simulation));

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

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Expected %d columns, got size %dx%d.', ncols, size(X,1), size(X,2));
    end
end
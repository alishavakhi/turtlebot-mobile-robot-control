%% ============================================================
% Export Task 1 simulation data after running Simulink
% File name: exported_task1_simulation_test1.mat
% ============================================================

clc;

modelName = 'task1_simulation';

if ~exist('out','var')
    error('Variable "out" not found. First run the Simulink model.');
end

if bdIsLoaded(modelName)
    modelFolder = fileparts(get_param(modelName,'FileName'));
else
    modelFolder = pwd;
end

Dsim_task1_simulation = struct();

%% Main pose signals
Dsim_task1_simulation.q_actual = forceCols(out.q_actual, 3);   % true simulated robot pose
Dsim_task1_simulation.q_est    = forceCols(out.q_est, 3);      % EKF/localizer estimate
Dsim_task1_simulation.q_target = forceCols(out.q_target, 3);

%% Diagnostic signals
Dsim_task1_simulation.q_est_imu = forceCols(out.q_est_imu, 3);

%% Control signals
Dsim_task1_simulation.v            = forceCol(out.v);
Dsim_task1_simulation.omega        = forceCol(out.omega);
Dsim_task1_simulation.omega_wheels = forceCols(out.omega_wheels, 2);
Dsim_task1_simulation.u_actual     = forceCols(out.u_actual, 2);  % [v_actual, omega_actual]

%% Time
Dsim_task1_simulation.t = forceCol(out.tout);

%% Make all main signals same length
N = min([
    size(Dsim_task1_simulation.q_actual,1)
    size(Dsim_task1_simulation.q_est,1)
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

%% Optional Vicon signal
% In simulation, Vicon may be disabled or scalar. If it is not N x 3,
% we store NaN values to keep the data structure consistent.
Dsim_task1_simulation.Y_viscon = optionalCols(out, 'Y_viscon', 3, N);

%% Save in the same folder as the Simulink file
savePath = fullfile(modelFolder, 'exported_task1_simulation_test1.mat');
save(savePath, 'Dsim_task1_simulation');

fprintf('\nSaved simulation data here:\n%s\n', savePath);

fprintf('\nSaved fields:\n');
disp(fieldnames(Dsim_task1_simulation));

fprintf('\nInitial q_actual:\n');
disp(Dsim_task1_simulation.q_actual(1,:));

fprintf('Final q_actual:\n');
disp(Dsim_task1_simulation.q_actual(end,:));

fprintf('\nInitial q_est:\n');
disp(Dsim_task1_simulation.q_est(1,:));

fprintf('Final q_est:\n');
disp(Dsim_task1_simulation.q_est(end,:));

fprintf('\nFinal position error to Task 1 goal:\n');
q_goal_task1 = [-1.60; -1.10; 0];
pos_err = norm(Dsim_task1_simulation.q_actual(end,1:2).' - q_goal_task1(1:2));
theta_err = abs(atan2(sin(q_goal_task1(3) - Dsim_task1_simulation.q_actual(end,3)), ...
                      cos(q_goal_task1(3) - Dsim_task1_simulation.q_actual(end,3))));
fprintf('Position error = %.4f m\n', pos_err);
fprintf('Theta error    = %.4f rad = %.2f deg\n', theta_err, rad2deg(theta_err));

%% ============================================================
% Local helper functions
% ============================================================

function X = getRaw(sig)
    if isa(sig,'timeseries')
        X = squeeze(sig.Data);
    elseif isnumeric(sig)
        X = squeeze(sig);
    elseif isstruct(sig) && isfield(sig,'signals')
        X = squeeze(sig.signals.values);
    else
        error('Unknown signal format.');
    end
end

function X = forceCols(sig, ncols)
    X = getRaw(sig);

    if isvector(X)
        X = X(:);
    end

    if size(X,2) ~= ncols && size(X,1) == ncols
        X = X.';
    end

    if size(X,2) ~= ncols
        error('Signal does not have expected number of columns. Expected %d columns, got %d.', ncols, size(X,2));
    end
end

function x = forceCol(sig)
    x = getRaw(sig);
    x = x(:);
end

function X = optionalCols(outObj, fieldName, ncols, N)
    try
        sig = outObj.(fieldName);
        X = getRaw(sig);

        if isvector(X)
            X = X(:);
        end

        if size(X,2) ~= ncols && size(X,1) == ncols
            X = X.';
        end

        if size(X,2) == ncols && size(X,1) >= N
            X = X(1:N,:);
        else
            X = NaN(N,ncols);
        end

    catch
        X = NaN(N,ncols);
    end
end
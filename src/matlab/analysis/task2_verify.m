clear; clc; close all;

fprintf('\n============================================================\n');
fprintf('VERIFY TASK 2 NUMBERS FOR REPORT\n');
fprintf('============================================================\n');

%% ---------- Files ----------
realFileCandidates = { ...
    'test3_task2_full_outputs.mat', ...
    'test3_task2_all_outputs.mat', ...
    'task2_real_outputs.mat'};

simFileCandidates = { ...
    'exported_task2_simulation.mat', ...
    'task2_simulation_outputs.mat'};

realFile = findExistingFile(realFileCandidates);
simFile  = findExistingFile(simFileCandidates);

fprintf('\nReal data file used:\n%s\n', realFile);
fprintf('\nSimulation data file used:\n%s\n', simFile);

%% ---------- Load real data ----------
Sreal = load(realFile);

if isfield(Sreal, 'D_task2')
    Dreal = Sreal.D_task2;
elseif isfield(Sreal, 'Dreal')
    Dreal = Sreal.Dreal;
elseif isfield(Sreal, 'D_task1')
    Dreal = Sreal.D_task1;
    warning('D_task2 not found. Using D_task1. Check if this is correct.');
else
    error('No recognized real data structure found.');
end

%% ---------- Load simulation data ----------
Ssim = load(simFile);

if isfield(Ssim, 'Dsim_task2')
    Dsim = Ssim.Dsim_task2;
elseif isfield(Ssim, 'Dsim')
    Dsim = Ssim.Dsim;
else
    error('No recognized simulation data structure found.');
end

%% ---------- Constants ----------
q_goal2 = [1.00; -0.30; 0];
omega_limit = 10.0;

%% ---------- Extract real signals ----------
q_est_real = getPose(Dreal, {'q_est', 'q_localizer'});
q_target_real = getPose(Dreal, {'q_target'});

q_vicon = getPose(Dreal, { ...
    'Y_viscon_ground_trith', ...
    'Y_vicon_ground_truth', ...
    'Y_viscon_ground_truth', ...
    'Y_viscon', ...
    'Y_vicon', ...
    'Y_vicon_used'});

Y_vicon_used = getPoseOptional(Dreal, { ...
    'Y_vicon_used', ...
    'Y_viscon', ...
    'Y_vicon', ...
    'Y_viscon_ground_trith', ...
    'Y_vicon_ground_truth'}, size(q_vicon,1));

t_real = getTime(Dreal, size(q_vicon,1), 0.04);

v_real = getVectorOptional(Dreal, {'v'}, length(t_real));
omega_real = getVectorOptional(Dreal, {'omega'}, length(t_real));

omega_wheels_real = getMatrixOptional(Dreal, {'omega_wheels'}, length(t_real), 2);
bot_wheelspeed_real = getMatrixOptional(Dreal, {'bot_wheelspeed'}, length(t_real), 2);

%% ---------- Extract simulation signals ----------
q_actual_sim = getPose(Dsim, {'q_actual', 'q_true'});
q_est_sim = getPose(Dsim, {'q_est', 'q_localizer'});
q_target_sim = getPose(Dsim, {'q_target'});
t_sim = getTime(Dsim, size(q_actual_sim,1), 0.04);

v_sim = getVectorOptional(Dsim, {'v'}, length(t_sim));
omega_sim = getVectorOptional(Dsim, {'omega'}, length(t_sim));
omega_wheels_sim = getMatrixOptional(Dsim, {'omega_wheels'}, length(t_sim), 2);

%% ---------- Align lengths ----------
Nreal = min([length(t_real), size(q_vicon,1), size(q_est_real,1), size(q_target_real,1), ...
             length(v_real), length(omega_real), size(omega_wheels_real,1), size(bot_wheelspeed_real,1), size(Y_vicon_used,1)]);

t_real = t_real(1:Nreal);
q_vicon = q_vicon(1:Nreal,:);
q_est_real = q_est_real(1:Nreal,:);
q_target_real = q_target_real(1:Nreal,:);
v_real = v_real(1:Nreal);
omega_real = omega_real(1:Nreal);
omega_wheels_real = omega_wheels_real(1:Nreal,:);
bot_wheelspeed_real = bot_wheelspeed_real(1:Nreal,:);
Y_vicon_used = Y_vicon_used(1:Nreal,:);

Nsim = min([length(t_sim), size(q_actual_sim,1), size(q_est_sim,1), size(q_target_sim,1), ...
            length(v_sim), length(omega_sim), size(omega_wheels_sim,1)]);

t_sim = t_sim(1:Nsim);
q_actual_sim = q_actual_sim(1:Nsim,:);
q_est_sim = q_est_sim(1:Nsim,:);
q_target_sim = q_target_sim(1:Nsim,:);
v_sim = v_sim(1:Nsim);
omega_sim = omega_sim(1:Nsim);
omega_wheels_sim = omega_wheels_sim(1:Nsim,:);

%% ---------- Wrap angles ----------
q_vicon(:,3) = wrapToPiLocal(q_vicon(:,3));
q_est_real(:,3) = wrapToPiLocal(q_est_real(:,3));
q_actual_sim(:,3) = wrapToPiLocal(q_actual_sim(:,3));
q_est_sim(:,3) = wrapToPiLocal(q_est_sim(:,3));

%% ---------- Goal errors ----------
real_final_pos_err = norm(q_vicon(end,1:2).' - q_goal2(1:2));
real_final_theta_err = abs(wrapToPiLocal(q_goal2(3) - q_vicon(end,3)));

sim_final_pos_err = norm(q_actual_sim(end,1:2).' - q_goal2(1:2));
sim_final_theta_err = abs(wrapToPiLocal(q_goal2(3) - q_actual_sim(end,3)));

%% ---------- Localization errors ----------
real_loc_pos_err = vecnorm(q_est_real(:,1:2) - q_vicon(:,1:2), 2, 2);
real_loc_theta_err = wrapToPiLocal(q_est_real(:,3) - q_vicon(:,3));

sim_loc_pos_err = vecnorm(q_est_sim(:,1:2) - q_actual_sim(:,1:2), 2, 2);
sim_loc_theta_err = wrapToPiLocal(q_est_sim(:,3) - q_actual_sim(:,3));

real_loc_rmse = sqrt(mean(real_loc_pos_err.^2, 'omitnan'));
real_loc_max = max(real_loc_pos_err, [], 'omitnan');

sim_loc_rmse = sqrt(mean(sim_loc_pos_err.^2, 'omitnan'));
sim_loc_max = max(sim_loc_pos_err, [], 'omitnan');

%% ---------- Red area error ----------
redArea = [-0.60, -1.50, 0.80, 1.60];  % [x y width height]
in_red_real = insideRect(q_vicon(:,1:2), redArea);
in_red_sim = insideRect(q_actual_sim(:,1:2), redArea);

real_red_rmse = sqrt(mean(real_loc_pos_err(in_red_real).^2, 'omitnan'));
real_red_max = max(real_loc_pos_err(in_red_real), [], 'omitnan');

sim_red_rmse = sqrt(mean(sim_loc_pos_err(in_red_sim).^2, 'omitnan'));
sim_red_max = max(sim_loc_pos_err(in_red_sim), [], 'omitnan');

%% ---------- Vicon loss ----------
lost_vicon = all(abs(Y_vicon_used) < 1e-12, 2);
vicon_loss_count = sum(lost_vicon);
vicon_loss_total = numel(lost_vicon);
vicon_loss_percent = 100 * vicon_loss_count / vicon_loss_total;

%% ---------- Wheel speed column check ----------
if all(isnan(bot_wheelspeed_real(:)))
    bot_wheelspeed_LR = bot_wheelspeed_real;
    fprintf('\nMeasured wheel speed not available or all NaN.\n');
else
    err_same = mean(vecnorm(bot_wheelspeed_real - omega_wheels_real, 2, 2), 'omitnan');
    err_swap = mean(vecnorm([bot_wheelspeed_real(:,2), bot_wheelspeed_real(:,1)] - omega_wheels_real, 2, 2), 'omitnan');

    if err_swap < err_same
        bot_wheelspeed_LR = [bot_wheelspeed_real(:,2), bot_wheelspeed_real(:,1)];
        fprintf('\nMeasured wheel-speed columns appear swapped. Using swapped order.\n');
    else
        bot_wheelspeed_LR = bot_wheelspeed_real;
        fprintf('\nMeasured wheel-speed columns appear correct. Using original order.\n');
    end
end

max_real_command_wheel = max(abs(omega_wheels_real(:)), [], 'omitnan');
max_measured_wheel = max(abs(bot_wheelspeed_LR(:)), [], 'omitnan');
max_sim_command_wheel = max(abs(omega_wheels_sim(:)), [], 'omitnan');

max_real_v = max(abs(v_real), [], 'omitnan');
max_real_omega = max(abs(omega_real), [], 'omitnan');

max_sim_v = max(abs(v_sim), [], 'omitnan');
max_sim_omega = max(abs(omega_sim), [], 'omitnan');

%% ---------- Print report-ready output ----------
fprintf('\n============================================================\n');
fprintf('TASK 2 REPORT-READY NUMBERS\n');
fprintf('============================================================\n');

fprintf('\n--- Duration ---\n');
fprintf('Real duration: %.4f s\n', t_real(end));
fprintf('Simulation duration/raw or exported: %.4f s\n', t_sim(end));

fprintf('\n--- Final pose and final errors ---\n');
fprintf('Real final Vicon pose:       [%.4f %.4f %.4f]\n', q_vicon(end,1), q_vicon(end,2), q_vicon(end,3));
fprintf('Real final EKF pose:         [%.4f %.4f %.4f]\n', q_est_real(end,1), q_est_real(end,2), q_est_real(end,3));
fprintf('Simulation final actual:     [%.4f %.4f %.4f]\n', q_actual_sim(end,1), q_actual_sim(end,2), q_actual_sim(end,3));
fprintf('Simulation final EKF:        [%.4f %.4f %.4f]\n', q_est_sim(end,1), q_est_sim(end,2), q_est_sim(end,3));

fprintf('\nReal final position error:       %.4f m\n', real_final_pos_err);
fprintf('Real final theta error:          %.4f rad = %.2f deg\n', real_final_theta_err, rad2deg(real_final_theta_err));
fprintf('Simulation final position error: %.4f m\n', sim_final_pos_err);
fprintf('Simulation final theta error:    %.4f rad = %.2f deg\n', sim_final_theta_err, rad2deg(sim_final_theta_err));

fprintf('\n--- Localization error ---\n');
fprintf('Real localization RMSE position: %.4f m\n', real_loc_rmse);
fprintf('Real localization max position:  %.4f m\n', real_loc_max);
fprintf('Simulation localization RMSE:    %.4f m\n', sim_loc_rmse);
fprintf('Simulation localization max:     %.4f m\n', sim_loc_max);

fprintf('\n--- Red-area localization error ---\n');
fprintf('Real red-area RMSE position:     %.4f m\n', real_red_rmse);
fprintf('Real red-area max position:      %.4f m\n', real_red_max);
fprintf('Simulation red-area RMSE:        %.4f m\n', sim_red_rmse);
fprintf('Simulation red-area max:         %.4f m\n', sim_red_max);

fprintf('\n--- Vicon loss during Task 2 ---\n');
fprintf('Vicon loss samples: %d / %d = %.2f %%\n', vicon_loss_count, vicon_loss_total, vicon_loss_percent);

fprintf('\n--- Control inputs ---\n');
fprintf('Real max |v|:       %.4f m/s\n', max_real_v);
fprintf('Real max |omega|:   %.4f rad/s\n', max_real_omega);
fprintf('Simulation max |v|: %.4f m/s\n', max_sim_v);
fprintf('Simulation max |omega|: %.4f rad/s\n', max_sim_omega);

fprintf('\n--- Wheel speeds ---\n');
fprintf('Wheel-speed command limit: %.2f rad/s\n', omega_limit);
fprintf('Max real commanded wheel speed: %.4f rad/s\n', max_real_command_wheel);
fprintf('Max measured wheel speed:       %.4f rad/s\n', max_measured_wheel);
fprintf('Max simulation wheel command:   %.4f rad/s\n', max_sim_command_wheel);

fprintf('\nCommand-level wheel-speed limit respected? ');
if max_real_command_wheel <= omega_limit + 1e-9
    fprintf('YES\n');
else
    fprintf('NO\n');
end

%% ---------- Save CSV summary ----------
Summary = table( ...
    t_real(end), t_sim(end), ...
    real_final_pos_err, real_final_theta_err, rad2deg(real_final_theta_err), ...
    sim_final_pos_err, sim_final_theta_err, rad2deg(sim_final_theta_err), ...
    real_loc_rmse, real_loc_max, sim_loc_rmse, sim_loc_max, ...
    real_red_rmse, real_red_max, sim_red_rmse, sim_red_max, ...
    vicon_loss_count, vicon_loss_total, vicon_loss_percent, ...
    max_real_v, max_real_omega, max_sim_v, max_sim_omega, ...
    max_real_command_wheel, max_measured_wheel, max_sim_command_wheel, ...
    'VariableNames', { ...
    'RealDuration_s','SimulationDuration_s', ...
    'RealFinalPositionError_m','RealFinalThetaError_rad','RealFinalThetaError_deg', ...
    'SimulationFinalPositionError_m','SimulationFinalThetaError_rad','SimulationFinalThetaError_deg', ...
    'RealLocalizationRMSE_m','RealLocalizationMaxError_m', ...
    'SimulationLocalizationRMSE_m','SimulationLocalizationMaxError_m', ...
    'RealRedAreaRMSE_m','RealRedAreaMaxError_m', ...
    'SimulationRedAreaRMSE_m','SimulationRedAreaMaxError_m', ...
    'ViconLossSamples','ViconTotalSamples','ViconLossPercent', ...
    'RealMaxAbsV_m_s','RealMaxAbsOmega_rad_s', ...
    'SimulationMaxAbsV_m_s','SimulationMaxAbsOmega_rad_s', ...
    'MaxRealCommandedWheel_rad_s','MaxMeasuredWheel_rad_s','MaxSimulationCommandedWheel_rad_s'} ...
);

writetable(Summary, 'task2_verified_report_numbers.csv');
fprintf('\nSaved: task2_verified_report_numbers.csv\n');

%% ============================================================
% Local functions
% ============================================================

function file = findExistingFile(candidates)
    file = '';
    for i = 1:numel(candidates)
        if isfile(candidates{i})
            file = candidates{i};
            return;
        end
    end
    error('None of these files were found:\n%s', strjoin(candidates, '\n'));
end

function X = forceNxM(X, M)
    X = squeeze(X);

    if isvector(X)
        X = X(:);
        if M ~= 1 && numel(X) == M
            X = X(:).';
        end
    end

    if size(X,2) ~= M && size(X,1) == M
        X = X.';
    end

    if size(X,2) ~= M
        error('Expected %d columns, got size %dx%d.', M, size(X,1), size(X,2));
    end
end

function q = getPose(S, names)
    for i = 1:numel(names)
        if isfield(S, names{i})
            q = forceNxM(S.(names{i}), 3);
            return;
        end
    end
    error('None of these pose fields found: %s', strjoin(names, ', '));
end

function q = getPoseOptional(S, names, N)
    for i = 1:numel(names)
        if isfield(S, names{i})
            q = forceNxM(S.(names{i}), 3);
            return;
        end
    end
    q = NaN(N,3);
end

function v = getVectorOptional(S, names, N)
    for i = 1:numel(names)
        if isfield(S, names{i})
            v = squeeze(S.(names{i}));
            v = v(:);
            return;
        end
    end
    v = NaN(N,1);
end

function X = getMatrixOptional(S, names, N, M)
    for i = 1:numel(names)
        if isfield(S, names{i})
            X = forceNxM(S.(names{i}), M);
            return;
        end
    end
    X = NaN(N,M);
end

function t = getTime(S, N, Ts)
    if isfield(S, 't')
        t = S.t(:);
    elseif isfield(S, 'tout')
        t = S.tout(:);
    else
        t = (0:N-1).' * Ts;
    end
    t = t - t(1);
end

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end

function tf = insideRect(P, rect)
    x = P(:,1);
    y = P(:,2);
    tf = x >= rect(1) & x <= rect(1)+rect(3) & ...
         y >= rect(2) & y <= rect(2)+rect(4);
end
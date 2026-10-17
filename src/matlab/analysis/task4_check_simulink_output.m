%% check_task4_simulink_output.m
% Check Task 4 Simulink outputs

clc;

%% Extract data from Simulink output object

qTarget = getSigData(out.q_target);
qTrue   = getSigData(out.q_actual);
qEst    = getSigData(out.q_est);

% Time vector
if isa(out.q_est, 'timeseries')
    t = out.q_est.Time;
elseif isstruct(out.q_est) && isfield(out.q_est, 'time')
    t = out.q_est.time;
elseif isprop(out, 'tout')
    t = out.tout;
else
    t = (0:size(qEst,1)-1)'*0.04;
end

% Make all pose data Nx3
qTarget = makeNx3(qTarget);
qTrue   = makeNx3(qTrue);
qEst    = makeNx3(qEst);

N = min([size(qTarget,1), size(qTrue,1), size(qEst,1), numel(t)]);

qTarget = qTarget(1:N,:);
qTrue   = qTrue(1:N,:);
qEst    = qEst(1:N,:);
t       = t(1:N);

%% XY plot

figure('Color','w');
hold on; grid on; axis equal;

plot(qTarget(:,1), qTarget(:,2), 'k--', 'LineWidth', 1.8);
plot(qTrue(:,1), qTrue(:,2), 'b-', 'LineWidth', 1.6);
plot(qEst(:,1), qEst(:,2), 'r--', 'LineWidth', 1.6);

rectangle('Position',[1.80 -0.20 0.40 0.40], ...
    'FaceColor',[0.85 0.85 0.85], ...
    'EdgeColor','k', ...
    'LineWidth',2);

legend('Reference','True trajectory','LiDAR-EKF estimate','Cube', ...
    'Location','bestoutside');

xlabel('x [m]');
ylabel('y [m]');
title('Simulink XY trajectory check');

xlim([0.7 2.8]);
ylim([-0.9 0.8]);

%% Error calculation

eTrackXY = sqrt(sum((qTrue(:,1:2) - qTarget(:,1:2)).^2, 2));
eLocXY   = sqrt(sum((qEst(:,1:2) - qTrue(:,1:2)).^2, 2));

eTrackTh = wrapPiLocal(qTarget(:,3) - qTrue(:,3));
eLocTh   = wrapPiLocal(qEst(:,3) - qTrue(:,3));

figure('Color','w');

subplot(2,1,1);
hold on; grid on;
plot(t, eTrackXY, 'b', 'LineWidth', 1.5);
plot(t, eLocXY, 'r--', 'LineWidth', 1.5);
ylabel('position error [m]');
legend('Tracking error','Localization error', 'Location','best');
title('Simulink position errors');

subplot(2,1,2);
hold on; grid on;
plot(t, rad2deg(eTrackTh), 'b', 'LineWidth', 1.5);
plot(t, rad2deg(eLocTh), 'r--', 'LineWidth', 1.5);
ylabel('angle error [deg]');
xlabel('time [s]');
legend('Tracking heading error','Localization heading error', 'Location','best');
title('Simulink orientation errors');

%% Numerical metrics

fprintf('\n===== Simulink Task 4 Metrics =====\n');

fprintf('Tracking RMSE XY         = %.4f m\n', sqrt(mean(eTrackXY.^2)));
fprintf('Tracking max XY          = %.4f m\n', max(eTrackXY));
fprintf('Tracking final XY        = %.4f m\n', eTrackXY(end));

fprintf('Localization RMSE XY     = %.4f m\n', sqrt(mean(eLocXY.^2)));
fprintf('Localization max XY      = %.4f m\n', max(eLocXY));
fprintf('Localization final XY    = %.4f m\n', eLocXY(end));

fprintf('Tracking RMSE theta      = %.3f deg\n', rad2deg(sqrt(mean(eTrackTh.^2))));
fprintf('Localization RMSE theta  = %.3f deg\n', rad2deg(sqrt(mean(eLocTh.^2))));
fprintf('Localization final theta = %.3f deg\n', rad2deg(eLocTh(end)));

%% LiDAR diagnostics

innovation = getSigData(out.innovation_RMS);
nValid     = getSigData(out.n_valid_lidar_sim);

innovation = innovation(:);
nValid = nValid(:);

M = min([numel(t), numel(innovation), numel(nValid)]);

tL = t(1:M);
innovation = innovation(1:M);
nValid = nValid(1:M);

fprintf('Mean LiDAR innovation    = %.4f m\n', mean(innovation, 'omitnan'));
fprintf('Max LiDAR innovation     = %.4f m\n', max(innovation, [], 'omitnan'));
fprintf('Mean valid LiDAR rays    = %.2f\n', mean(nValid, 'omitnan'));

figure('Color','w');

subplot(2,1,1);
plot(tL, innovation, 'LineWidth', 1.4);
grid on;
ylabel('RMS innovation [m]');
title('LiDAR innovation');

subplot(2,1,2);
plot(tL, nValid, 'LineWidth', 1.4);
grid on;
ylabel('valid LiDAR rays');
xlabel('time [s]');
title('Number of LiDAR rays used in EKF update');

%% Helper functions

function X = getSigData(sig)

    if isa(sig, 'timeseries')
        X = sig.Data;

    elseif isstruct(sig)

        if isfield(sig, 'signals')
            X = sig.signals.values;
        elseif isfield(sig, 'Data')
            X = sig.Data;
        elseif isfield(sig, 'data')
            X = sig.data;
        else
            X = sig;
        end

    else
        X = sig;
    end

end

function X = makeNx3(X)

    X = squeeze(X);

    if size(X,2) ~= 3 && size(X,1) == 3
        X = X';
    end

end

function a = wrapPiLocal(a)

    a = atan2(sin(a), cos(a));

end
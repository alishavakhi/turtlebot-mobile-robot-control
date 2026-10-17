%% check_task4_wrong_ekf_initialization.m
% Check whether Task 4 Simulink really starts with:
% q_true0 ~= qhat0 in the EKF localization sense.

clc;

%% Read signals from Simulink output
qTrue = localGetData(out.q_actual);
qEst  = localGetData(out.q_est);
qRef  = localGetData(out.q_target);

qTrue = localMakeNx3(qTrue);
qEst  = localMakeNx3(qEst);
qRef  = localMakeNx3(qRef);

N = min([size(qTrue,1), size(qEst,1), size(qRef,1)]);
qTrue = qTrue(1:N,:);
qEst  = qEst(1:N,:);
qRef  = qRef(1:N,:);

%% Initial errors
eLoc0 = norm(qEst(1,1:2) - qTrue(1,1:2));
eLocTh0 = rad2deg(localWrapPi(qEst(1,3) - qTrue(1,3)));

eTrack0 = norm(qTrue(1,1:2) - qRef(1,1:2));
eTrackTh0 = rad2deg(localWrapPi(qRef(1,3) - qTrue(1,3)));

fprintf('\nInitial checks from Simulink logs:\n');
fprintf('q_ref(0)  = [%.4f, %.4f, %.2f deg]\n', ...
    qRef(1,1), qRef(1,2), rad2deg(qRef(1,3)));
fprintf('q_true(0) = [%.4f, %.4f, %.2f deg]\n', ...
    qTrue(1,1), qTrue(1,2), rad2deg(qTrue(1,3)));
fprintf('q_est(0)  = [%.4f, %.4f, %.2f deg]\n', ...
    qEst(1,1), qEst(1,2), rad2deg(qEst(1,3)));

fprintf('\nInitial tracking position error     = %.4f m\n', eTrack0);
fprintf('Initial tracking heading error      = %.2f deg\n', eTrackTh0);
fprintf('Initial localization position error = %.4f m\n', eLoc0);
fprintf('Initial localization heading error  = %.2f deg\n', eLocTh0);

%% Expected result for wrong-EKF-initialization case
fprintf('\nExpected for wrong EKF initialization:\n');
fprintf('Initial tracking error should be close to 0 m.\n');
fprintf('Initial localization error should be close to 0.0707 m.\n');
fprintf('Initial localization heading error should be close to 5 deg.\n');

if eLoc0 > 0.05
    fprintf('\nSTATUS: OK. EKF starts from a wrong initial estimate.\n');
else
    fprintf('\nSTATUS: NOT OK. EKF still seems initialized close to q_true.\n');
    fprintf('Check EKF q_init input and q_est Unit Delay initial condition.\n');
end

%% ============================================================
% Local helper functions
% ============================================================

function X = localGetData(sig)

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

    X = squeeze(X);

end

function X = localMakeNx3(X)

    X = squeeze(X);

    if size(X,2) ~= 3 && size(X,1) == 3
        X = X';
    end

end

function a = localWrapPi(a)

    a = atan2(sin(a), cos(a));

end
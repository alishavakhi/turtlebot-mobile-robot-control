%% main_task4_sim_only_combined_v5_labels.m
% Robotics Laboratory - Final Activity - Task 4 Simulation Only
% No Vicon is used.
% Localization: Odometry/IMU prediction + LiDAR EKF correction
% Control: nonlinear trajectory tracking using q_hat
%
% Version v5:
% - Starts from a pose close to the final configuration of Task 2
% - Uses small artificial sensor perturbations for non-ideal simulation testing
% - Uses robust LiDAR innovation gating
% - Uses a minimum number of valid LiDAR rays before EKF update
% - Runs exact-initial-point and non-exact-initial-point EKF cases
% - Saves ONE combined set of figures where exact and non-exact initial-point cases are
%   shown together in every plot.
% - Saves a detailed numerical report for checking performance beside plots.

clear; clc; close all;

setDefaultTask4Style();

%% =========================
%  1) Parameters
% ==========================
cfg.randomSeed = 4;
cfg.Ts = 0.04;                 % [s]
cfg.r  = 0.03;                 % [m] wheel radius
cfg.d  = 0.165;                % [m] wheel distance
cfg.wM = 10;                   % [rad/s] wheel speed saturation

% Map / task coordinates
cfg.x1  = -2.90;
cfg.x2  = -1.60;
cfg.x3  = -1.50;
cfg.x4  = -1.40;
cfg.x5  = -0.60;
cfg.x6  =  0.20;
cfg.x7  =  1.00;
cfg.x8  =  1.45;
cfg.x9  =  1.80;
cfg.x10 =  2.20;
cfg.x11 =  2.90;

cfg.y1 = -1.50;
cfg.y2 = -1.10;
cfg.y3 = -0.70;
cfg.y4 = -0.30;
cfg.y5 = -0.20;
cfg.y6 =  0.10;
cfg.y7 =  0.20;
cfg.y8 =  1.50;

% Cube model used for LiDAR intersection
cfg.cube_xmin = cfg.x9;
cfg.cube_xmax = cfg.x10;
cfg.cube_ymin = cfg.y5;
cfg.cube_ymax = cfg.y7;
cfg.cube_center = [2.00; 0.00];

% Task 4 starts from a configuration close to the final configuration of Task 2.
% A small offset is used because the exact final pose of Task 2 is not fixed here.
cfg.q0_true = [0.96; -0.34; deg2rad(4)];

% Start of the circular trajectory, same as Task 3
cfg.p_circle_start = [cfg.x8; cfg.y4];

% Circle parameters
cfg.R_circle = norm(cfg.p_circle_start - cfg.cube_center);
cfg.Omega_circle = 0.30;                 % [rad/s]
cfg.T_circle = 3*pi/cfg.Omega_circle;    % one and a half turns

% Approach phase from near Task 2 final pose to circle start
cfg.T_approach = 8.0;                    % [s]

% Controller gains
cfg.kx  = 0.75;
cfg.ky  = 2.00;
cfg.kth = 1.20;

% LiDAR parameters
cfg.lidar_offset = 0.076;                % [m] translated along sagittal axis
cfg.lidar_angles = linspace(-pi, pi, 37)';
cfg.lidar_angles(end) = [];              % 36 rays, 360 degrees
cfg.lidar_max_range = 3.0;               % [m]

% Small artificial simulation perturbations.
% These are not professor-given values. They are only for non-ideal simulation testing.
cfg.sigma_lidar = 0.006;                 % [m]
cfg.p_lidar_dropout = 0.03;              % probability of missing ray

% Encoder / IMU noise
cfg.sigma_encoder = 0.0008;              % [rad] wheel angle increment noise
cfg.sigma_imu_w   = 0.010;               % [rad/s] yaw-rate noise

% True motion uncertainty
cfg.sigma_v_true = 0.004;                % relative noise on true v
cfg.sigma_w_true = 0.004;                % additive noise on true omega

% EKF covariance
cfg.Q = diag([1.5e-5, 1.5e-5, 6e-5]);   % process covariance
cfg.P0_nominal   = diag([1e-4, 1e-4, deg2rad(1)^2]);
cfg.P0_perturbed = diag([5e-3, 5e-3, deg2rad(7)^2]);

% Robust LiDAR update settings
cfg.min_lidar_rays_for_update = 3;
cfg.lidar_innovation_gate = 0.08;        % [m]

% Plot style
cfg.style = getTask4PlotStyle();

outDir = "task4_sim_outputs_combined_v5";
if ~exist(outDir, "dir")
    mkdir(outDir);
end

%% =========================
%  2) Build reference trajectory
% ==========================
ref = buildTask4Reference(cfg);

fprintf('\nReference trajectory created.\n');
fprintf('Initial true pose   = [%.3f, %.3f, %.2f deg]\n', ...
    cfg.q0_true(1), cfg.q0_true(2), rad2deg(cfg.q0_true(3)));
fprintf('Circle radius       = %.4f m\n', cfg.R_circle);
fprintf('Circle omega        = %.4f rad/s\n', cfg.Omega_circle);
fprintf('Circle v_d          = %.4f m/s\n', cfg.R_circle*cfg.Omega_circle);
fprintf('Circle duration     = %.2f s\n', cfg.T_circle);
fprintf('Total duration      = %.2f s\n\n', ref.t(end));

%% =========================
%  3) Run exact and non-exact initial-point simulations
% ==========================
scenarios(1).name = "exact_initial_point";
scenarios(1).label = "Exact initial point";
scenarios(1).qhat0_error = [0; 0; 0];
scenarios(1).P0 = cfg.P0_nominal;

scenarios(2).name = "non_exact_initial_point";
scenarios(2).label = "Non-exact initial point";
scenarios(2).qhat0_error = [0.05; -0.05; deg2rad(5)];
scenarios(2).P0 = cfg.P0_perturbed;

results = cell(numel(scenarios),1);
metricsAll = table();

for i = 1:numel(scenarios)

    % Reset the random seed before each scenario so the two cases are compared
    % under the same noise realization as much as possible.
    rng(cfg.randomSeed);

    fprintf('Running scenario: %s\n', scenarios(i).name);

    results{i} = runTask4Scenario(ref, cfg, scenarios(i));

    printMetrics(results{i}, scenarios(i).name);

    metricsRow = metricsToTable(results{i}.metrics, scenarios(i).name);
    metricsAll = [metricsAll; metricsRow]; %#ok<AGROW>

end

makeTask4CombinedPlots(results, scenarios, ref, cfg, outDir);

metricsFile = fullfile(outDir, "task4_metrics_summary.csv");
writetable(metricsAll, metricsFile);

[detailedMetricsFile, numericalReportFile] = buildTask4NumericalReport(results, scenarios, ref, cfg, outDir);

fprintf('\nDone.\n');
fprintf('Combined figures saved in folder: %s\n', outDir);
fprintf('Metrics saved in: %s\n', metricsFile);
fprintf('Detailed metrics saved in: %s\n', detailedMetricsFile);
fprintf('Numerical report saved in: %s\n\n', numericalReportFile);

%% ========================================================================
%                              FUNCTIONS
% ========================================================================

function setDefaultTask4Style()

    set(groot, 'defaultFigureColor', 'w');
    set(groot, 'defaultAxesColor', 'w');
    set(groot, 'defaultAxesXColor', 'k');
    set(groot, 'defaultAxesYColor', 'k');
    set(groot, 'defaultTextColor', 'k');
    set(groot, 'defaultAxesFontName', 'Times New Roman');
    set(groot, 'defaultTextFontName', 'Times New Roman');
    set(groot, 'defaultAxesFontSize', 12);
    set(groot, 'defaultLineLineWidth', 1.4);

end

function style = getTask4PlotStyle()

    % Consistent with previous task plotting style
    style.cRef = [0.0000 0.0000 0.0000];      % reference / black

    style.cNomTrue = [0.0000 0.5000 0.0000];  % nominal ground truth / green
    style.cNomEKF  = [0.8500 0.3250 0.0980];  % nominal EKF / orange

    style.cPerTrue = [0.0000 0.4470 0.7410];  % perturbed ground truth / blue
    style.cPerEKF  = [0.4940 0.1840 0.5560];  % perturbed EKF / purple

    style.cTrackNom = [0.0000 0.5000 0.0000];
    style.cTrackPer = [0.0000 0.4470 0.7410];
    style.cLocNom   = [0.8500 0.3250 0.0980];
    style.cLocPer   = [0.4940 0.1840 0.5560];

    style.cStart = [0.0000 0.6000 0.0000];
    style.cEnd   = [0.6000 0.0000 0.6000];

    style.gridColor = [0.78 0.78 0.78];
    style.mapColor  = [0.15 0.15 0.15];
    style.cubeFace  = [0.86 0.86 0.86];
    style.cubeEdge  = [0.00 0.00 0.00];

    style.figW = 1150;
    style.figH = 820;

end

function ref = buildTask4Reference(cfg)

    Ts = cfg.Ts;

    % Circle initial angle
    phi0 = atan2(cfg.p_circle_start(2) - cfg.cube_center(2), ...
                 cfg.p_circle_start(1) - cfg.cube_center(1));

    % Counter-clockwise circular motion
    theta_circle0 = wrapPi(phi0 + pi/2);

    q_start = cfg.q0_true;
    q_circle_start = [cfg.p_circle_start; theta_circle0];

    % Approach trajectory using cubic Hermite path + smooth timing
    refA = hermiteApproachReference(q_start, q_circle_start, cfg.T_approach, Ts);

    % Circle trajectory
    tC = (Ts:Ts:cfg.T_circle)';
    phi = phi0 + cfg.Omega_circle*tC;

    xC = cfg.cube_center(1) + cfg.R_circle*cos(phi);
    yC = cfg.cube_center(2) + cfg.R_circle*sin(phi);
    thC = wrapPi(phi + pi/2);

    vC = cfg.R_circle*cfg.Omega_circle*ones(size(tC));
    wC = cfg.Omega_circle*ones(size(tC));

    refC.t = refA.t(end) + tC;
    refC.q = [xC, yC, thC];
    refC.v = vC;
    refC.w = wC;

    % Concatenate
    ref.t = [refA.t; refC.t];
    ref.q = [refA.q; refC.q];
    ref.v = [refA.v; refC.v];
    ref.w = [refA.w; refC.w];

end

function ref = hermiteApproachReference(qi, qf, T, Ts)

    t = (0:Ts:T)';
    tau = t/T;

    % Smooth timing law: s(0)=0, s(T)=1, sdot(0)=sdot(T)=0
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    sdot = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
    sddot = (60*tau - 180*tau.^2 + 120*tau.^3)/(T^2);

    pi2 = qi(1:2)';
    pf2 = qf(1:2)';

    L = max(norm(pf2 - pi2), 0.45);
    m0 = L*[cos(qi(3)), sin(qi(3))];
    m1 = L*[cos(qf(3)), sin(qf(3))];

    h00 =  2*s.^3 - 3*s.^2 + 1;
    h10 =      s.^3 - 2*s.^2 + s;
    h01 = -2*s.^3 + 3*s.^2;
    h11 =      s.^3 -   s.^2;

    dh00 =  6*s.^2 - 6*s;
    dh10 =  3*s.^2 - 4*s + 1;
    dh01 = -6*s.^2 + 6*s;
    dh11 =  3*s.^2 - 2*s;

    d2h00 =  12*s - 6;
    d2h10 =   6*s - 4;
    d2h01 = -12*s + 6;
    d2h11 =   6*s - 2;

    p = h00.*pi2 + h10.*m0 + h01.*pf2 + h11.*m1;
    dpds = dh00.*pi2 + dh10.*m0 + dh01.*pf2 + dh11.*m1;
    d2pds2 = d2h00.*pi2 + d2h10.*m0 + d2h01.*pf2 + d2h11.*m1;

    pdot = dpds .* sdot;
    pddot = d2pds2.*(sdot.^2) + dpds.*sddot; %#ok<NASGU>

    theta = atan2(dpds(:,2), dpds(:,1));
    theta_unwrapped = unwrap(theta);
    theta = wrapPi(theta_unwrapped);

    v = sqrt(sum(pdot.^2, 2));

    % Omega from numerical derivative of continuous theta
    omega = gradient(theta_unwrapped, Ts);

    % Avoid numerical spikes at very low speed
    omega(v < 1e-4) = 0;

    ref.t = t;
    ref.q = [p, theta];
    ref.v = v;
    ref.w = omega;

end

function result = runTask4Scenario(ref, cfg, scenario)

    N = numel(ref.t);

    qTrue = zeros(3,N);
    qHat  = zeros(3,N);
    Psave = zeros(3,3,N);

    vCmd = zeros(N,1);
    wCmd = zeros(N,1);
    wLcmd = zeros(N,1);
    wRcmd = zeros(N,1);

    innovationRMS = nan(N,1);
    nValidLidar = zeros(N,1);

    qTrue(:,1) = cfg.q0_true;
    qHat(:,1)  = wrapState(cfg.q0_true + scenario.qhat0_error);

    P = scenario.P0;
    Psave(:,:,1) = P;

    for k = 1:N-1

        qd = ref.q(k,:)';
        vd = ref.v(k);
        wd = ref.w(k);

        % Controller uses qHat only
        [v_des, w_des] = trackingController(qHat(:,k), qd, vd, wd, cfg);

        % Apply wheel saturation
        [v_sat, w_sat, wL, wR] = saturateVW(v_des, w_des, cfg);

        vCmd(k) = v_sat;
        wCmd(k) = w_sat;
        wLcmd(k) = wL;
        wRcmd(k) = wR;

        % True robot motion: small actuation uncertainty
        v_true = v_sat*(1 + cfg.sigma_v_true*randn);
        w_true = w_sat + cfg.sigma_w_true*randn;

        qTrue(:,k+1) = exactUnicycleStep(qTrue(:,k), v_true, w_true, cfg.Ts);

        % Encoder and IMU measurements
        [v_enc, w_enc, w_imu] = simulateOdometrySensors(v_true, w_true, cfg);

        % Use encoder linear speed and fused angular speed
        v_odo = v_enc;
        w_odo = 0.45*w_enc + 0.55*w_imu;

        % EKF prediction
        [qPred, PPred] = ekfPredict(qHat(:,k), P, v_odo, w_odo, cfg);

        % Simulated LiDAR measurement from true pose
        zLidar = simulateLidar(qTrue(:,k+1), cfg);

        % EKF update with LiDAR
        [qHat(:,k+1), P, innovationRMS(k+1), nValidLidar(k+1)] = ...
            ekfUpdateLidar(qPred, PPred, zLidar, cfg);

        Psave(:,:,k+1) = P;

    end

    vCmd(end) = vCmd(end-1);
    wCmd(end) = wCmd(end-1);
    wLcmd(end) = wLcmd(end-1);
    wRcmd(end) = wRcmd(end-1);

    result.t = ref.t;
    result.qTrue = qTrue;
    result.qHat = qHat;
    result.Psave = Psave;
    result.vCmd = vCmd;
    result.wCmd = wCmd;
    result.wLcmd = wLcmd;
    result.wRcmd = wRcmd;
    result.innovationRMS = innovationRMS;
    result.nValidLidar = nValidLidar;

    % Metrics
    eTrackXY = sqrt(sum((qTrue(1:2,:)' - ref.q(:,1:2)).^2, 2));
    eTrackTh = wrapPi(ref.q(:,3) - qTrue(3,:)');

    eLocXY = sqrt(sum((qHat(1:2,:)' - qTrue(1:2,:)').^2, 2));
    eLocTh = wrapPi(qHat(3,:)' - qTrue(3,:)');

    result.metrics.track_rmse_xy = sqrt(mean(eTrackXY.^2));
    result.metrics.track_max_xy  = max(eTrackXY);
    result.metrics.track_rmse_th = sqrt(mean(eTrackTh.^2));

    result.metrics.loc_rmse_xy = sqrt(mean(eLocXY.^2));
    result.metrics.loc_max_xy  = max(eLocXY);
    result.metrics.loc_rmse_th = sqrt(mean(eLocTh.^2));

    result.metrics.final_tracking_xy = eTrackXY(end);
    result.metrics.final_localization_xy = eLocXY(end);

    result.eTrackXY = eTrackXY;
    result.eTrackTh = eTrackTh;
    result.eLocXY = eLocXY;
    result.eLocTh = eLocTh;

end

function [v, w] = trackingController(qhat, qd, vd, wd, cfg)

    dx = qd(1) - qhat(1);
    dy = qd(2) - qhat(2);

    th = qhat(3);

    % Error in robot/body frame
    ex =  cos(th)*dx + sin(th)*dy;
    ey = -sin(th)*dx + cos(th)*dy;
    eth = wrapPi(qd(3) - qhat(3));

    % Nonlinear tracking controller
    v = vd*cos(eth) + cfg.kx*ex;
    w = wd + cfg.ky*vd*ey + cfg.kth*sin(eth);

end

function [vSat, wSat, wL, wR] = saturateVW(v, w, cfg)

    wL = v/cfg.r - cfg.d*w/(2*cfg.r);
    wR = v/cfg.r + cfg.d*w/(2*cfg.r);

    wL = min(max(wL, -cfg.wM), cfg.wM);
    wR = min(max(wR, -cfg.wM), cfg.wM);

    vSat = cfg.r*(wL + wR)/2;
    wSat = cfg.r*(wR - wL)/cfg.d;

end

function qnext = exactUnicycleStep(q, v, w, Ts)

    x = q(1);
    y = q(2);
    th = q(3);

    if abs(w) < 1e-8
        xnext = x + Ts*v*cos(th);
        ynext = y + Ts*v*sin(th);
    else
        xnext = x + (v/w)*(sin(th + w*Ts) - sin(th));
        ynext = y - (v/w)*(cos(th + w*Ts) - cos(th));
    end

    thnext = wrapPi(th + Ts*w);

    qnext = [xnext; ynext; thnext];

end

function [v_enc, w_enc, w_imu] = simulateOdometrySensors(v_true, w_true, cfg)

    wL_true = v_true/cfg.r - cfg.d*w_true/(2*cfg.r);
    wR_true = v_true/cfg.r + cfg.d*w_true/(2*cfg.r);

    dphiL = wL_true*cfg.Ts + cfg.sigma_encoder*randn;
    dphiR = wR_true*cfg.Ts + cfg.sigma_encoder*randn;

    v_enc = cfg.r*(dphiL + dphiR)/(2*cfg.Ts);
    w_enc = cfg.r*(dphiR - dphiL)/(cfg.d*cfg.Ts);

    w_imu = w_true + cfg.sigma_imu_w*randn;

end

function [qPred, PPred] = ekfPredict(q, P, v, w, cfg)

    Ts = cfg.Ts;
    th = q(3);
    th_mid = th + 0.5*Ts*w;

    qPred = zeros(3,1);
    qPred(1) = q(1) + Ts*v*cos(th_mid);
    qPred(2) = q(2) + Ts*v*sin(th_mid);
    qPred(3) = wrapPi(q(3) + Ts*w);

    F = eye(3);
    F(1,3) = -Ts*v*sin(th_mid);
    F(2,3) =  Ts*v*cos(th_mid);

    PPred = F*P*F' + cfg.Q;
    PPred = 0.5*(PPred + PPred');

end

function z = simulateLidar(qTrue, cfg)

    z = lidarModel(qTrue, cfg);

    valid = isfinite(z);

    z(valid) = z(valid) + cfg.sigma_lidar*randn(sum(valid),1);
    z(valid) = max(z(valid), 0.02);

    dropout = rand(numel(z),1) < cfg.p_lidar_dropout;
    z(dropout) = NaN;

end

function h = lidarModel(q, cfg)

    x = q(1);
    y = q(2);
    th = q(3);

    % LiDAR center translated along sagittal axis
    pL = [x; y] + cfg.lidar_offset*[cos(th); sin(th)];

    n = numel(cfg.lidar_angles);
    h = nan(n,1);

    for i = 1:n
        alpha = th + cfg.lidar_angles(i);
        h(i) = rayBoxDistance(pL, alpha, cfg);
    end

end

function dist = rayBoxDistance(p, alpha, cfg)

    xmin = cfg.cube_xmin;
    xmax = cfg.cube_xmax;
    ymin = cfg.cube_ymin;
    ymax = cfg.cube_ymax;

    dir = [cos(alpha); sin(alpha)];
    cand = [];

    % Intersections with vertical cube edges
    if abs(dir(1)) > 1e-10

        tx1 = (xmin - p(1))/dir(1);
        yx1 = p(2) + tx1*dir(2);

        if tx1 > 1e-6 && yx1 >= ymin && yx1 <= ymax
            cand(end+1) = tx1; %#ok<AGROW>
        end

        tx2 = (xmax - p(1))/dir(1);
        yx2 = p(2) + tx2*dir(2);

        if tx2 > 1e-6 && yx2 >= ymin && yx2 <= ymax
            cand(end+1) = tx2; %#ok<AGROW>
        end

    end

    % Intersections with horizontal cube edges
    if abs(dir(2)) > 1e-10

        ty1 = (ymin - p(2))/dir(2);
        xy1 = p(1) + ty1*dir(1);

        if ty1 > 1e-6 && xy1 >= xmin && xy1 <= xmax
            cand(end+1) = ty1; %#ok<AGROW>
        end

        ty2 = (ymax - p(2))/dir(2);
        xy2 = p(1) + ty2*dir(1);

        if ty2 > 1e-6 && xy2 >= ymin && xy2 <= ymax
            cand(end+1) = ty2; %#ok<AGROW>
        end

    end

    if isempty(cand)
        dist = NaN;
    else
        dist = min(cand);

        if dist > cfg.lidar_max_range
            dist = NaN;
        end
    end

end

function [qUpd, PUpd, innovationRMS, nValid] = ekfUpdateLidar(qPred, PPred, z, cfg)

    hPred = lidarModel(qPred, cfg);
    Hfull = numericalLidarJacobian(qPred, cfg);

    valid = isfinite(z) & isfinite(hPred) & all(isfinite(Hfull),2);

    nValid = sum(valid);

    if nValid < cfg.min_lidar_rays_for_update
        qUpd = qPred;
        PUpd = PPred;
        innovationRMS = NaN;
        return;
    end

    zv = z(valid);
    hv = hPred(valid);
    H  = Hfull(valid,:);

    innov = zv - hv;

    % Innovation gating: reject inconsistent LiDAR rays
    gate = abs(innov) < cfg.lidar_innovation_gate;

    zv = zv(gate); %#ok<NASGU>
    H  = H(gate,:);
    innov = innov(gate);

    nValid = numel(innov);

    if nValid < cfg.min_lidar_rays_for_update
        qUpd = qPred;
        PUpd = PPred;
        innovationRMS = NaN;
        return;
    end

    innovationRMS = sqrt(mean(innov.^2));

    R = cfg.sigma_lidar^2 * eye(numel(innov));

    S = H*PPred*H' + R;

    % Numerical safety
    S = 0.5*(S + S') + 1e-10*eye(size(S));

    K = (PPred*H')/S;

    qUpd = qPred + K*innov;
    qUpd = wrapState(qUpd);

    I = eye(3);
    PUpd = (I - K*H)*PPred*(I - K*H)' + K*R*K';
    PUpd = 0.5*(PUpd + PUpd');

end

function H = numericalLidarJacobian(q, cfg)

    epsVal = [1e-5; 1e-5; 1e-5];
    h0 = lidarModel(q, cfg);
    H = nan(numel(h0), 3);

    for j = 1:3

        qp = q;
        qm = q;

        qp(j) = qp(j) + epsVal(j);
        qm(j) = qm(j) - epsVal(j);

        qp = wrapState(qp);
        qm = wrapState(qm);

        hp = lidarModel(qp, cfg);
        hm = lidarModel(qm, cfg);

        H(:,j) = (hp - hm)/(2*epsVal(j));

    end

end


function [detailedMetricsFile, numericalReportFile] = buildTask4NumericalReport(results, scenarios, ref, cfg, outDir)

    % This function creates numerical outputs that can be checked beside the plots.
    % It is useful for deciding whether the EKF behavior is acceptable before
    % transferring the algorithm to Simulink.

    nSc = numel(results);

    Scenario = strings(nSc,1);
    Duration_s = zeros(nSc,1);

    Initial_Localization_xy_m = zeros(nSc,1);
    Initial_Localization_theta_deg = zeros(nSc,1);

    Tracking_RMSE_xy_m = zeros(nSc,1);
    Tracking_Max_xy_m = zeros(nSc,1);
    Tracking_Final_xy_m = zeros(nSc,1);
    Tracking_RMSE_theta_deg = zeros(nSc,1);
    Tracking_Final_theta_deg = zeros(nSc,1);

    Localization_RMSE_xy_m = zeros(nSc,1);
    Localization_Max_xy_m = zeros(nSc,1);
    Localization_Final_xy_m = zeros(nSc,1);
    Localization_RMSE_theta_deg = zeros(nSc,1);
    Localization_Final_theta_deg = zeros(nSc,1);

    Localization_Settling_Time_2cm_s = nan(nSc,1);
    Heading_Settling_Time_2deg_s = nan(nSc,1);

    Innovation_Mean_m = zeros(nSc,1);
    Innovation_Max_m = zeros(nSc,1);
    Valid_LiDAR_Rays_Mean = zeros(nSc,1);
    Valid_LiDAR_Rays_Min = zeros(nSc,1);
    Valid_LiDAR_Rays_Max = zeros(nSc,1);
    EKF_Update_Skipped_percent = zeros(nSc,1);

    Max_Abs_v_mps = zeros(nSc,1);
    Max_Abs_omega_radps = zeros(nSc,1);
    Max_Abs_Wheel_Speed_radps = zeros(nSc,1);
    Wheel_Saturation_percent = zeros(nSc,1);

    Final_True_x_m = zeros(nSc,1);
    Final_True_y_m = zeros(nSc,1);
    Final_True_theta_deg = zeros(nSc,1);
    Final_EKF_x_m = zeros(nSc,1);
    Final_EKF_y_m = zeros(nSc,1);
    Final_EKF_theta_deg = zeros(nSc,1);

    for i = 1:nSc

        res = results{i};
        t = res.t;

        Scenario(i) = string(scenarios(i).name);
        Duration_s(i) = t(end);

        Initial_Localization_xy_m(i) = res.eLocXY(1);
        Initial_Localization_theta_deg(i) = abs(rad2deg(res.eLocTh(1)));

        Tracking_RMSE_xy_m(i) = res.metrics.track_rmse_xy;
        Tracking_Max_xy_m(i) = res.metrics.track_max_xy;
        Tracking_Final_xy_m(i) = res.metrics.final_tracking_xy;
        Tracking_RMSE_theta_deg(i) = rad2deg(res.metrics.track_rmse_th);
        Tracking_Final_theta_deg(i) = abs(rad2deg(res.eTrackTh(end)));

        Localization_RMSE_xy_m(i) = res.metrics.loc_rmse_xy;
        Localization_Max_xy_m(i) = res.metrics.loc_max_xy;
        Localization_Final_xy_m(i) = res.metrics.final_localization_xy;
        Localization_RMSE_theta_deg(i) = rad2deg(res.metrics.loc_rmse_th);
        Localization_Final_theta_deg(i) = abs(rad2deg(res.eLocTh(end)));

        Localization_Settling_Time_2cm_s(i) = settlingTime(t, res.eLocXY, 0.02, 1.0);
        Heading_Settling_Time_2deg_s(i) = settlingTime(t, abs(rad2deg(res.eLocTh)), 2.0, 1.0);

        Innovation_Mean_m(i) = nanMeanLocal(res.innovationRMS);
        Innovation_Max_m(i) = nanMaxLocal(res.innovationRMS);
        Valid_LiDAR_Rays_Mean(i) = mean(res.nValidLidar);
        Valid_LiDAR_Rays_Min(i) = min(res.nValidLidar);
        Valid_LiDAR_Rays_Max(i) = max(res.nValidLidar);
        EKF_Update_Skipped_percent(i) = 100*mean(res.nValidLidar < cfg.min_lidar_rays_for_update);

        Max_Abs_v_mps(i) = max(abs(res.vCmd));
        Max_Abs_omega_radps(i) = max(abs(res.wCmd));
        Max_Abs_Wheel_Speed_radps(i) = max([abs(res.wLcmd); abs(res.wRcmd)]);
        Wheel_Saturation_percent(i) = 100*mean((abs(res.wLcmd) >= 0.99*cfg.wM) | ...
                                               (abs(res.wRcmd) >= 0.99*cfg.wM));

        Final_True_x_m(i) = res.qTrue(1,end);
        Final_True_y_m(i) = res.qTrue(2,end);
        Final_True_theta_deg(i) = rad2deg(res.qTrue(3,end));
        Final_EKF_x_m(i) = res.qHat(1,end);
        Final_EKF_y_m(i) = res.qHat(2,end);
        Final_EKF_theta_deg(i) = rad2deg(res.qHat(3,end));

    end

    detailedTable = table(Scenario, Duration_s, ...
        Initial_Localization_xy_m, Initial_Localization_theta_deg, ...
        Tracking_RMSE_xy_m, Tracking_Max_xy_m, Tracking_Final_xy_m, ...
        Tracking_RMSE_theta_deg, Tracking_Final_theta_deg, ...
        Localization_RMSE_xy_m, Localization_Max_xy_m, Localization_Final_xy_m, ...
        Localization_RMSE_theta_deg, Localization_Final_theta_deg, ...
        Localization_Settling_Time_2cm_s, Heading_Settling_Time_2deg_s, ...
        Innovation_Mean_m, Innovation_Max_m, ...
        Valid_LiDAR_Rays_Mean, Valid_LiDAR_Rays_Min, Valid_LiDAR_Rays_Max, ...
        EKF_Update_Skipped_percent, ...
        Max_Abs_v_mps, Max_Abs_omega_radps, Max_Abs_Wheel_Speed_radps, ...
        Wheel_Saturation_percent, ...
        Final_True_x_m, Final_True_y_m, Final_True_theta_deg, ...
        Final_EKF_x_m, Final_EKF_y_m, Final_EKF_theta_deg);

    detailedMetricsFile = fullfile(outDir, "task4_detailed_metrics.csv");
    writetable(detailedTable, detailedMetricsFile);

    numericalReportFile = fullfile(outDir, "task4_numerical_report.txt");
    fid = fopen(numericalReportFile, 'w');

    fprintf(fid, 'Numerical report - LiDAR EKF simulation\\n');
    fprintf(fid, '================================================\\n\\n');

    fprintf(fid, 'Simulation setup\\n');
    fprintf(fid, '----------------\\n');
    fprintf(fid, 'Sampling time: %.4f s\\n', cfg.Ts);
    fprintf(fid, 'Initial true pose: [%.3f, %.3f, %.2f deg]\\n', ...
        cfg.q0_true(1), cfg.q0_true(2), rad2deg(cfg.q0_true(3)));
    fprintf(fid, 'Circle center: [%.3f, %.3f] m\\n', cfg.cube_center(1), cfg.cube_center(2));
    fprintf(fid, 'Circle radius: %.4f m\\n', cfg.R_circle);
    fprintf(fid, 'Circle angular rate: %.4f rad/s\\n', cfg.Omega_circle);
    fprintf(fid, 'Total duration: %.2f s\\n\\n', ref.t(end));

    fprintf(fid, 'Noise and EKF assumptions\\n');
    fprintf(fid, '-------------------------\\n');
    fprintf(fid, 'The following noise values are artificial simulation assumptions, not values specified in the task statement.\\n');
    fprintf(fid, 'LiDAR sigma: %.4f m\\n', cfg.sigma_lidar);
    fprintf(fid, 'LiDAR dropout probability: %.2f %%\\n', 100*cfg.p_lidar_dropout);
    fprintf(fid, 'Encoder increment sigma: %.5f rad\\n', cfg.sigma_encoder);
    fprintf(fid, 'IMU yaw-rate sigma: %.4f rad/s\\n', cfg.sigma_imu_w);
    fprintf(fid, 'LiDAR innovation gate: %.3f m\\n', cfg.lidar_innovation_gate);
    fprintf(fid, 'Minimum valid LiDAR rays for EKF update: %d\\n\\n', cfg.min_lidar_rays_for_update);

    for i = 1:nSc

        fprintf(fid, '%s\\n', char(Scenario(i)));
        fprintf(fid, '%s\\n', repmat('-', 1, strlength(Scenario(i))));
        fprintf(fid, 'Initial localization error: %.4f m, %.2f deg\\n', ...
            Initial_Localization_xy_m(i), Initial_Localization_theta_deg(i));
        fprintf(fid, 'Tracking RMSE: %.4f m, %.2f deg\\n', ...
            Tracking_RMSE_xy_m(i), Tracking_RMSE_theta_deg(i));
        fprintf(fid, 'Tracking max/final position error: %.4f m / %.4f m\\n', ...
            Tracking_Max_xy_m(i), Tracking_Final_xy_m(i));
        fprintf(fid, 'Localization RMSE: %.4f m, %.2f deg\\n', ...
            Localization_RMSE_xy_m(i), Localization_RMSE_theta_deg(i));
        fprintf(fid, 'Localization max/final position error: %.4f m / %.4f m\\n', ...
            Localization_Max_xy_m(i), Localization_Final_xy_m(i));
        fprintf(fid, 'Final localization heading error: %.2f deg\\n', ...
            Localization_Final_theta_deg(i));

        if isnan(Localization_Settling_Time_2cm_s(i))
            fprintf(fid, 'Localization settling time below 2 cm: not reached\\n');
        else
            fprintf(fid, 'Localization settling time below 2 cm: %.2f s\\n', ...
                Localization_Settling_Time_2cm_s(i));
        end

        fprintf(fid, 'Mean/max LiDAR innovation: %.4f m / %.4f m\\n', ...
            Innovation_Mean_m(i), Innovation_Max_m(i));
        fprintf(fid, 'Mean valid LiDAR rays: %.2f, skipped EKF updates: %.2f %%\\n', ...
            Valid_LiDAR_Rays_Mean(i), EKF_Update_Skipped_percent(i));
        fprintf(fid, 'Max wheel speed: %.3f rad/s, wheel saturation samples: %.2f %%\\n', ...
            Max_Abs_Wheel_Speed_radps(i), Wheel_Saturation_percent(i));
        fprintf(fid, 'Final true pose: [%.3f, %.3f, %.2f deg]\\n', ...
            Final_True_x_m(i), Final_True_y_m(i), Final_True_theta_deg(i));
        fprintf(fid, 'Final EKF pose:  [%.3f, %.3f, %.2f deg]\\n\\n', ...
            Final_EKF_x_m(i), Final_EKF_y_m(i), Final_EKF_theta_deg(i));

    end

    fclose(fid);

end

function ts = settlingTime(t, e, threshold, window_s)

    % Returns the first time after which the error stays below threshold for
    % at least window_s seconds. Returns NaN if this condition is never met.

    if isempty(t) || isempty(e)
        ts = NaN;
        return;
    end

    dt = median(diff(t));
    win = max(1, round(window_s/dt));
    e = e(:);

    ts = NaN;

    for k = 1:(numel(e)-win+1)
        if all(e(k:k+win-1) < threshold)
            ts = t(k);
            return;
        end
    end

end

function m = nanMeanLocal(x)

    x = x(isfinite(x));
    if isempty(x)
        m = NaN;
    else
        m = mean(x);
    end

end

function m = nanMaxLocal(x)

    x = x(isfinite(x));
    if isempty(x)
        m = NaN;
    else
        m = max(x);
    end

end

function makeTask4CombinedPlots(results, scenarios, ref, cfg, outDir)

    t = results{1}.t;
    style = cfg.style;

    nom = results{1};
    per = results{2};

    %% =========================
    %  Figure 1: XY map, exact and non-exact initial points together
    % ==========================
    fig = figure('Name', 'XY map - exact vs non-exact initial point', 'Color','w');
    fig.Position = [80 80 style.figW style.figH];

    hold on; grid on; axis equal;
    styleAxes(gca, cfg);

    drawSimpleMap(cfg);

    hRef = plot(ref.q(:,1), ref.q(:,2), 'k--', 'LineWidth', 1.9);

    hNomTrue = plot(nom.qTrue(1,:), nom.qTrue(2,:), '-.', ...
        'Color', style.cNomTrue, 'LineWidth', 1.8);
    hNomEKF = plot(nom.qHat(1,:), nom.qHat(2,:), '--', ...
        'Color', style.cNomEKF, 'LineWidth', 1.7);

    hPerTrue = plot(per.qTrue(1,:), per.qTrue(2,:), '-', ...
        'Color', style.cPerTrue, 'LineWidth', 1.6);
    hPerEKF = plot(per.qHat(1,:), per.qHat(2,:), ':', ...
        'Color', style.cPerEKF, 'LineWidth', 2.1);

    hStart = plot(cfg.q0_true(1), cfg.q0_true(2), 'o', ...
        'MarkerFaceColor', style.cStart, ...
        'MarkerEdgeColor', style.cStart, ...
        'MarkerSize', 8);

    hEnd = plot(ref.q(end,1), ref.q(end,2), 'o', ...
        'MarkerFaceColor', style.cEnd, ...
        'MarkerEdgeColor', style.cEnd, ...
        'MarkerSize', 8);

    xlabel('x [m]');
    ylabel('y [m]');
    title('XY trajectory - exact vs non-exact initial point');

    lgd = legend([hRef, hNomTrue, hNomEKF, hPerTrue, hPerEKF, hStart, hEnd], ...
        {'Reference', 'Exact true', 'Exact EKF', ...
         'Non-exact true', 'Non-exact EKF', 'Start', 'End'}, ...
        'Location','bestoutside');
    styleLegend(lgd);

    setTask4XYLimits(gca, cfg, ref, results);

    saveFig(fullfile(outDir, "task4_01_xy_map_exact_vs_non_exact.pdf"));

    %% =========================
    %  Figure 2: States, exact and non-exact initial points together
    % ==========================
    fig = figure('Name', 'States - exact vs non-exact initial point', 'Color','w');
    fig.Position = [100 100 style.figW style.figH];

    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    thetaRefPlot  = unwrap(ref.q(:,3));
    thetaNomTrue  = unwrap(nom.qTrue(3,:)');
    thetaNomHat   = unwrap(nom.qHat(3,:)');
    thetaPerTrue  = unwrap(per.qTrue(3,:)');
    thetaPerHat   = unwrap(per.qHat(3,:)');

    nexttile;
    hold on; grid on;
    plot(t, ref.q(:,1), 'k--', 'LineWidth', 1.6);
    plot(t, nom.qTrue(1,:), '-.', 'Color', style.cNomTrue, 'LineWidth', 1.6);
    plot(t, nom.qHat(1,:), '--', 'Color', style.cNomEKF, 'LineWidth', 1.5);
    plot(t, per.qTrue(1,:), '-', 'Color', style.cPerTrue, 'LineWidth', 1.5);
    plot(t, per.qHat(1,:), ':', 'Color', style.cPerEKF, 'LineWidth', 2.0);
    ylabel('x [m]');
    title('Longitudinal position');
    styleAxes(gca, cfg);
    lgd = legend('Reference','Exact true','Exact EKF','Non-exact true','Non-exact EKF', ...
        'Location','bestoutside');
    styleLegend(lgd);

    nexttile;
    hold on; grid on;
    plot(t, ref.q(:,2), 'k--', 'LineWidth', 1.6);
    plot(t, nom.qTrue(2,:), '-.', 'Color', style.cNomTrue, 'LineWidth', 1.6);
    plot(t, nom.qHat(2,:), '--', 'Color', style.cNomEKF, 'LineWidth', 1.5);
    plot(t, per.qTrue(2,:), '-', 'Color', style.cPerTrue, 'LineWidth', 1.5);
    plot(t, per.qHat(2,:), ':', 'Color', style.cPerEKF, 'LineWidth', 2.0);
    ylabel('y [m]');
    title('Lateral position');
    styleAxes(gca, cfg);

    nexttile;
    hold on; grid on;
    plot(t, thetaRefPlot, 'k--', 'LineWidth', 1.6);
    plot(t, thetaNomTrue, '-.', 'Color', style.cNomTrue, 'LineWidth', 1.6);
    plot(t, thetaNomHat, '--', 'Color', style.cNomEKF, 'LineWidth', 1.5);
    plot(t, thetaPerTrue, '-', 'Color', style.cPerTrue, 'LineWidth', 1.5);
    plot(t, thetaPerHat, ':', 'Color', style.cPerEKF, 'LineWidth', 2.0);
    ylabel('\theta [rad]');
    xlabel('time [s]');
    title('Orientation, unwrapped for visualization');
    styleAxes(gca, cfg);

    sgtitle('State comparison - exact vs non-exact initial point');

    saveFig(fullfile(outDir, "task4_02_states_exact_vs_non_exact.pdf"));

    %% =========================
    %  Figure 3: Errors, exact and non-exact initial points together
    % ==========================
    fig = figure('Name', 'Errors - exact vs non-exact initial point', 'Color','w');
    fig.Position = [120 120 style.figW style.figH];

    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    hold on; grid on;
    h1 = plot(t, nom.eTrackXY, '-', 'Color', style.cTrackNom, 'LineWidth', 1.7);
    h2 = plot(t, nom.eLocXY, '--', 'Color', style.cLocNom, 'LineWidth', 1.7);
    h3 = plot(t, per.eTrackXY, '-', 'Color', style.cTrackPer, 'LineWidth', 1.7);
    h4 = plot(t, per.eLocXY, '--', 'Color', style.cLocPer, 'LineWidth', 1.7);
    ylabel('position error [m]');
    title('Position errors');
    styleAxes(gca, cfg);
    lgd = legend([h1 h2 h3 h4], ...
        {'Exact tracking','Exact localization','Non-exact tracking','Non-exact localization'}, ...
        'Location','bestoutside');
    styleLegend(lgd);

    nexttile;
    hold on; grid on;
    plot(t, rad2deg(nom.eTrackTh), '-', 'Color', style.cTrackNom, 'LineWidth', 1.7);
    plot(t, rad2deg(nom.eLocTh), '--', 'Color', style.cLocNom, 'LineWidth', 1.7);
    plot(t, rad2deg(per.eTrackTh), '-', 'Color', style.cTrackPer, 'LineWidth', 1.7);
    plot(t, rad2deg(per.eLocTh), '--', 'Color', style.cLocPer, 'LineWidth', 1.7);
    ylabel('angle error [deg]');
    xlabel('time [s]');
    title('Orientation errors');
    styleAxes(gca, cfg);

    sgtitle('Error comparison - exact vs non-exact initial point');

    saveFig(fullfile(outDir, "task4_03_errors_exact_vs_non_exact.pdf"));

    %% =========================
    %  Figure 4: Inputs, exact and non-exact initial points together
    % ==========================
    fig = figure('Name', 'Inputs - exact vs non-exact initial point', 'Color','w');
    fig.Position = [140 140 style.figW style.figH];

    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    hold on; grid on;
    plot(t, nom.vCmd, '-', 'Color', style.cNomTrue, 'LineWidth', 1.7);
    plot(t, nom.wCmd, '--', 'Color', style.cNomEKF, 'LineWidth', 1.7);
    plot(t, per.vCmd, '-', 'Color', style.cPerTrue, 'LineWidth', 1.7);
    plot(t, per.wCmd, '--', 'Color', style.cPerEKF, 'LineWidth', 1.7);
    ylabel('control input');
    title('Kinematic control inputs');
    styleAxes(gca, cfg);
    lgd = legend('Exact v [m/s]','Exact \omega [rad/s]', ...
                 'Non-exact v [m/s]','Non-exact \omega [rad/s]', ...
                 'Location','bestoutside');
    styleLegend(lgd);

    nexttile;
    hold on; grid on;
    plot(t, nom.wLcmd, '-', 'Color', style.cNomTrue, 'LineWidth', 1.7);
    plot(t, nom.wRcmd, '--', 'Color', style.cNomEKF, 'LineWidth', 1.7);
    plot(t, per.wLcmd, '-', 'Color', style.cPerTrue, 'LineWidth', 1.7);
    plot(t, per.wRcmd, '--', 'Color', style.cPerEKF, 'LineWidth', 1.7);
    yline(cfg.wM, 'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
    yline(-cfg.wM, 'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
    ylabel('wheel speed [rad/s]');
    xlabel('time [s]');
    title('Wheel speed commands');
    styleAxes(gca, cfg);
    lgd = legend('Exact \omega_L','Exact \omega_R', ...
                 'Non-exact \omega_L','Non-exact \omega_R', ...
                 'Location','bestoutside');
    styleLegend(lgd);

    sgtitle('Input comparison - exact vs non-exact initial point');

    saveFig(fullfile(outDir, "task4_04_inputs_exact_vs_non_exact.pdf"));

    %% =========================
    %  Figure 5: LiDAR innovation, exact and non-exact initial points together
    % ==========================
    fig = figure('Name', 'LiDAR-EKF - exact vs non-exact initial point', 'Color','w');
    fig.Position = [160 160 style.figW style.figH];

    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    hold on; grid on;
    plot(t, nom.innovationRMS, '-', 'Color', style.cNomEKF, 'LineWidth', 1.7);
    plot(t, per.innovationRMS, '--', 'Color', style.cPerEKF, 'LineWidth', 1.7);
    ylabel('RMS innovation [m]');
    title('LiDAR innovation after gating');
    styleAxes(gca, cfg);
    lgd = legend('Exact initial point','Non-exact initial point', 'Location','best');
    styleLegend(lgd);

    nexttile;
    hold on; grid on;
    plot(t, nom.nValidLidar, '-', 'Color', style.cNomEKF, 'LineWidth', 1.7);
    plot(t, per.nValidLidar, '--', 'Color', style.cPerEKF, 'LineWidth', 1.7);
    ylabel('valid LiDAR rays');
    xlabel('time [s]');
    title('Number of LiDAR rays used in EKF update');
    styleAxes(gca, cfg);
    lgd = legend('Exact initial point','Non-exact initial point', 'Location','best');
    styleLegend(lgd);

    sgtitle('LiDAR-EKF comparison - exact vs non-exact initial point');

    saveFig(fullfile(outDir, "task4_05_lidar_innovation_exact_vs_non_exact.pdf"));

end

function drawSimpleMap(cfg)

    style = cfg.style;

    % Cube
    rectangle('Position', [cfg.cube_xmin, cfg.cube_ymin, ...
                           cfg.cube_xmax-cfg.cube_xmin, ...
                           cfg.cube_ymax-cfg.cube_ymin], ...
        'FaceColor', style.cubeFace, ...
        'EdgeColor', style.cubeEdge, ...
        'LineWidth', 2.0, ...
        'HandleVisibility','off');

    plot(cfg.cube_center(1), cfg.cube_center(2), 'kx', ...
        'LineWidth', 2, ...
        'MarkerSize', 10, ...
        'HandleVisibility','off');

end

function setTask4XYLimits(ax, cfg, ref, resultsCell)

    allX = [ref.q(:,1); cfg.cube_xmin; cfg.cube_xmax; cfg.q0_true(1)];
    allY = [ref.q(:,2); cfg.cube_ymin; cfg.cube_ymax; cfg.q0_true(2)];

    for i = 1:numel(resultsCell)

        if isempty(resultsCell{i})
            continue;
        end

        result = resultsCell{i};

        allX = [allX; result.qTrue(1,:)'; result.qHat(1,:)']; %#ok<AGROW>
        allY = [allY; result.qTrue(2,:)'; result.qHat(2,:)']; %#ok<AGROW>

    end

    padX = 0.20;
    padY = 0.20;

    xlim(ax, [min(allX)-padX, max(allX)+padX]);
    ylim(ax, [min(allY)-padY, max(allY)+padY]);

end

function styleAxes(ax, cfg)

    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = cfg.style.gridColor;
    ax.GridAlpha = 0.45;
    ax.LineWidth = 0.8;
    ax.FontName = 'Times New Roman';
    ax.FontSize = 12;
    ax.Box = 'on';

end

function styleLegend(lgd)

    lgd.Color = 'w';
    lgd.TextColor = 'k';
    lgd.EdgeColor = [0.75 0.75 0.75];
    lgd.FontName = 'Times New Roman';
    lgd.FontSize = 11;

end

function printMetrics(result, scenarioName)

    fprintf('\nMetrics for %s:\n', scenarioName);
    fprintf('  Tracking RMSE position      = %.4f m\n', result.metrics.track_rmse_xy);
    fprintf('  Tracking max position error = %.4f m\n', result.metrics.track_max_xy);
    fprintf('  Tracking RMSE heading       = %.2f deg\n', rad2deg(result.metrics.track_rmse_th));
    fprintf('  Localization RMSE position  = %.4f m\n', result.metrics.loc_rmse_xy);
    fprintf('  Localization max position   = %.4f m\n', result.metrics.loc_max_xy);
    fprintf('  Localization RMSE heading   = %.2f deg\n', rad2deg(result.metrics.loc_rmse_th));
    fprintf('  Final tracking position     = %.4f m\n', result.metrics.final_tracking_xy);
    fprintf('  Final localization position = %.4f m\n\n', result.metrics.final_localization_xy);

end

function T = metricsToTable(metrics, scenarioName)

    T = table();
    T.Scenario = string(scenarioName);
    T.Tracking_RMSE_xy_m = metrics.track_rmse_xy;
    T.Tracking_Max_xy_m = metrics.track_max_xy;
    T.Tracking_RMSE_theta_deg = rad2deg(metrics.track_rmse_th);
    T.Localization_RMSE_xy_m = metrics.loc_rmse_xy;
    T.Localization_Max_xy_m = metrics.loc_max_xy;
    T.Localization_RMSE_theta_deg = rad2deg(metrics.loc_rmse_th);
    T.Final_Tracking_xy_m = metrics.final_tracking_xy;
    T.Final_Localization_xy_m = metrics.final_localization_xy;

end

function saveFig(filename)

    filename = string(filename);

    try

        exportgraphics(gcf, filename, 'ContentType','vector');

        [folder, name, ~] = fileparts(char(filename));
        pngName = fullfile(folder, [name '.png']);
        exportgraphics(gcf, pngName, 'Resolution', 300);

    catch ME

        warning('Could not export with exportgraphics. Using saveas instead. Error: %s', ME.message);
        saveas(gcf, filename);

    end

end

function q = wrapState(q)

    q(3) = wrapPi(q(3));

end

function a = wrapPi(a)

    a = atan2(sin(a), cos(a));

end
%% task4_extract_final_report_numbers.m
% Extract final Task 4 metrics from exact and non-exact Simulink outputs.

clear; clc;

exactFile = "out.simulink.exact.mat";
nonExactFile = "out.simulink.non.exact.mat";

S_exact = load(exactFile);
outExact = S_exact.out;

S_non = load(nonExactFile);
outNon = S_non.out;

exact = packOutput(outExact, "Exact initial point");
nonExact = packOutput(outNon, "Non-exact initial point");

exact = computeMetrics(exact);
nonExact = computeMetrics(nonExact);

writeTextReport("task4_final_authoritative_metrics.txt", exact, nonExact);
writeLatexTable("task4_final_authoritative_table.tex", exact, nonExact);

disp("Done. Files created:");
disp("task4_final_authoritative_metrics.txt");
disp("task4_final_authoritative_table.tex");

%% ============================================================
% Local functions
% ============================================================

function R = packOutput(out, caseName)

    R.caseName = caseName;

    R.t = out.tout(:);

    R.q_target = makeNx3(getData(out.q_target));
    R.q_actual = makeNx3(getData(out.q_actual));
    R.q_est    = makeNx3(getData(out.q_est));

    R.v = getData(out.v);
    R.v = R.v(:);

    R.omega = getData(out.omega);
    R.omega = R.omega(:);

    R.omega_wheels = makeNx2(getData(out.omega_wheels));

    R.innovation_RMS = getData(out.innovation_RMS);
    R.innovation_RMS = R.innovation_RMS(:);

    if isprop(out, "n_valid_lidar_sim")
        R.n_valid_lidar = getData(out.n_valid_lidar_sim);
    elseif isprop(out, "n_valid_lidar")
        R.n_valid_lidar = getData(out.n_valid_lidar);
    else
        error("No n_valid_lidar signal found.");
    end
    R.n_valid_lidar = R.n_valid_lidar(:);

    N = min([ ...
        numel(R.t), ...
        size(R.q_target,1), ...
        size(R.q_actual,1), ...
        size(R.q_est,1), ...
        numel(R.v), ...
        numel(R.omega), ...
        size(R.omega_wheels,1), ...
        numel(R.innovation_RMS), ...
        numel(R.n_valid_lidar)]);

    R.t = R.t(1:N);
    R.q_target = R.q_target(1:N,:);
    R.q_actual = R.q_actual(1:N,:);
    R.q_est = R.q_est(1:N,:);
    R.v = R.v(1:N);
    R.omega = R.omega(1:N);
    R.omega_wheels = R.omega_wheels(1:N,:);
    R.innovation_RMS = R.innovation_RMS(1:N);
    R.n_valid_lidar = R.n_valid_lidar(1:N);

end

function R = computeMetrics(R)

    % Use only the report time interval shown in the final plots.
    idx = R.t <= 40;
    idxFinal = find(idx, 1, "last");

    eTrackXY = vecnorm(R.q_actual(:,1:2) - R.q_target(:,1:2), 2, 2);
    eLocXY   = vecnorm(R.q_est(:,1:2) - R.q_actual(:,1:2), 2, 2);

    eTrackTh = wrapPiLocal(R.q_target(:,3) - R.q_actual(:,3));
    eLocTh   = wrapPiLocal(R.q_est(:,3) - R.q_actual(:,3));

    R.m.duration = R.t(idxFinal);

    R.m.initial_track_xy = eTrackXY(1);
    R.m.initial_loc_xy = eLocXY(1);
    R.m.initial_track_heading = rad2deg(eTrackTh(1));
    R.m.initial_loc_heading = rad2deg(eLocTh(1));

    R.m.track_rmse_xy = sqrt(mean(eTrackXY(idx).^2));
    R.m.track_max_xy = max(eTrackXY(idx));
    R.m.track_final_xy = eTrackXY(idxFinal);

    R.m.loc_rmse_xy = sqrt(mean(eLocXY(idx).^2));
    R.m.loc_max_xy = max(eLocXY(idx));
    R.m.loc_final_xy = eLocXY(idxFinal);

    R.m.track_rmse_heading = rad2deg(sqrt(mean(eTrackTh(idx).^2)));
    R.m.loc_rmse_heading = rad2deg(sqrt(mean(eLocTh(idx).^2)));

    R.m.max_loc_heading = max(abs(rad2deg(eLocTh(idx))));
    R.m.max_track_heading = max(abs(rad2deg(eTrackTh(idx))));

    R.m.max_abs_v = max(abs(R.v(idx)));
    R.m.max_abs_omega = max(abs(R.omega(idx)));
    R.m.max_abs_wheel_speed = max(abs(R.omega_wheels(idx,:)), [], "all");

    R.m.mean_innovation = mean(R.innovation_RMS(idx), "omitnan");
    R.m.max_innovation = max(R.innovation_RMS(idx), [], "omitnan");

    R.m.mean_valid_rays = mean(R.n_valid_lidar(idx), "omitnan");
    R.m.min_valid_rays = min(R.n_valid_lidar(idx), [], "omitnan");
    R.m.max_valid_rays = max(R.n_valid_lidar(idx), [], "omitnan");

    R.m.valid_update_percent = 100 * mean(~isnan(R.innovation_RMS(idx)));
    R.m.skipped_update_percent = 100 - R.m.valid_update_percent;

end

function writeTextReport(fileName, exact, nonExact)

    fid = fopen(fileName, "w");

    writeOne(fid, exact);
    fprintf(fid, "\n");
    writeOne(fid, nonExact);

    fclose(fid);

end

function writeOne(fid, R)

    fprintf(fid, "Metrics for %s:\n", R.caseName);
    fprintf(fid, "Trajectory duration [s] = %.2f\n", R.m.duration);
    fprintf(fid, "Initial tracking position error [m] = %.4f\n", R.m.initial_track_xy);
    fprintf(fid, "Initial localization position error [m] = %.4f\n", R.m.initial_loc_xy);
    fprintf(fid, "Initial tracking heading error [deg] = %.2f\n", R.m.initial_track_heading);
    fprintf(fid, "Initial localization heading error [deg] = %.2f\n", R.m.initial_loc_heading);
    fprintf(fid, "Tracking RMSE position [m] = %.4f\n", R.m.track_rmse_xy);
    fprintf(fid, "Tracking max position error [m] = %.4f\n", R.m.track_max_xy);
    fprintf(fid, "Tracking final position error [m] = %.4f\n", R.m.track_final_xy);
    fprintf(fid, "Tracking RMSE heading [deg] = %.2f\n", R.m.track_rmse_heading);
    fprintf(fid, "Localization RMSE position [m] = %.4f\n", R.m.loc_rmse_xy);
    fprintf(fid, "Localization max position error [m] = %.4f\n", R.m.loc_max_xy);
    fprintf(fid, "Localization final position error [m] = %.4f\n", R.m.loc_final_xy);
    fprintf(fid, "Localization RMSE heading [deg] = %.2f\n", R.m.loc_rmse_heading);
    fprintf(fid, "Maximum localization heading error [deg] = %.2f\n", R.m.max_loc_heading);
    fprintf(fid, "Maximum |v| [m/s] = %.4f\n", R.m.max_abs_v);
    fprintf(fid, "Maximum |omega| [rad/s] = %.4f\n", R.m.max_abs_omega);
    fprintf(fid, "Maximum commanded wheel speed [rad/s] = %.4f\n", R.m.max_abs_wheel_speed);
    fprintf(fid, "Mean LiDAR innovation [m] = %.4f\n", R.m.mean_innovation);
    fprintf(fid, "Max LiDAR innovation [m] = %.4f\n", R.m.max_innovation);
    fprintf(fid, "Mean valid LiDAR rays = %.2f\n", R.m.mean_valid_rays);
    fprintf(fid, "Min/Max valid LiDAR rays = %.0f / %.0f\n", R.m.min_valid_rays, R.m.max_valid_rays);
    fprintf(fid, "Valid LiDAR update percentage [%%] = %.2f\n", R.m.valid_update_percent);
    fprintf(fid, "Skipped LiDAR update percentage [%%] = %.2f\n", R.m.skipped_update_percent);

end

function writeLatexTable(fileName, exact, nonExact)

    fid = fopen(fileName, "w");

    fprintf(fid, "\\begin{table}[H]\n");
    fprintf(fid, "\\centering\n");
    fprintf(fid, "\\caption{Numerical summary of Task 4 simulation results.}\n");
    fprintf(fid, "\\label{tab:task4_results}\n");
    fprintf(fid, "\\begin{tabular}{lll}\n");
    fprintf(fid, "\\hline\n");
    fprintf(fid, "Metric & Exact initial point & Non-exact initial point \\\\\n");
    fprintf(fid, "\\hline\n");
    fprintf(fid, "Trajectory duration [s] & %.2f & %.2f \\\\\n", exact.m.duration, nonExact.m.duration);
    fprintf(fid, "Tracking RMSE position [m] & %.4f & %.4f \\\\\n", exact.m.track_rmse_xy, nonExact.m.track_rmse_xy);
    fprintf(fid, "Tracking max position error [m] & %.4f & %.4f \\\\\n", exact.m.track_max_xy, nonExact.m.track_max_xy);
    fprintf(fid, "Localization RMSE position [m] & %.4f & %.4f \\\\\n", exact.m.loc_rmse_xy, nonExact.m.loc_rmse_xy);
    fprintf(fid, "Maximum localization error [m] & %.4f & %.4f \\\\\n", exact.m.loc_max_xy, nonExact.m.loc_max_xy);
    fprintf(fid, "Final localization error [m] & %.4f & %.4f \\\\\n", exact.m.loc_final_xy, nonExact.m.loc_final_xy);
    fprintf(fid, "Maximum localization heading error [deg] & %.2f & %.2f \\\\\n", exact.m.max_loc_heading, nonExact.m.max_loc_heading);
    fprintf(fid, "Maximum commanded wheel speed [rad/s] & %.4f & %.4f \\\\\n", exact.m.max_abs_wheel_speed, nonExact.m.max_abs_wheel_speed);
    fprintf(fid, "Mean LiDAR innovation [m] & %.4f & %.4f \\\\\n", exact.m.mean_innovation, nonExact.m.mean_innovation);
    fprintf(fid, "Valid LiDAR update percentage [\\%%] & %.2f & %.2f \\\\\n", exact.m.valid_update_percent, nonExact.m.valid_update_percent);
    fprintf(fid, "\\hline\n");
    fprintf(fid, "\\end{tabular}\n");
    fprintf(fid, "\\end{table}\n");

    fclose(fid);

end

function X = getData(sig)

    if isa(sig, "timeseries")
        X = sig.Data;
    elseif isstruct(sig)
        if isfield(sig, "signals")
            X = sig.signals.values;
        elseif isfield(sig, "Data")
            X = sig.Data;
        else
            X = sig;
        end
    else
        X = sig;
    end

    X = squeeze(X);

end

function X = makeNx3(X)

    X = squeeze(X);
    if size(X,2) ~= 3 && size(X,1) == 3
        X = X';
    end

end

function W = makeNx2(W)

    W = squeeze(W);
    if size(W,2) ~= 2 && size(W,1) == 2
        W = W';
    end

end

function a = wrapPiLocal(a)

    a = atan2(sin(a), cos(a));

end
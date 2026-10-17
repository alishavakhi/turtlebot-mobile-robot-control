clear; clc;

%% ============================================================
% Split ONLY Task 1 from three real robot tests
%
% Important correction:
% q0 is selected as the FIRST VALID VICON SAMPLE INSIDE THE
% official initial-position area, not simply the first nonzero Vicon sample.
%
% Works with:
%   realrobot_test1.mat
%   realrobot_test2.mat
%   realrobot_test3.mat
%
% Outputs:
%   test1_task1_separated.mat
%   test2_task1_separated.mat
%   test3_task1_separated.mat
%   task1_three_real_runs_summary.csv
%   task1_three_real_runs_latex_table.txt
%% ============================================================

rawFiles = {
    'realrobot_test1.mat'
    'realrobot_test2.mat'
    'realrobot_test3.mat'
};

exportedFiles = {
    'exported_realrobot_test1.mat'
    'exported_realrobot_test2.mat'
    'exported_realrobot_test3.mat'
};

outFiles = {
    'test1_task1_separated.mat'
    'test2_task1_separated.mat'
    'test3_task1_separated.mat'
};

%% Task 1 goal
q_goal_task1 = [-1.60, -1.10, 0.00];

%% Official initial-position area
x1 = -2.90;
x3 = -1.50;
y6 =  0.10;
y8 =  1.50;

%% Thresholds
posTol   = 0.05;
thetaTol = 0.05;
TsDefault = 0.04;

%% Manual fallback indices from previous split scripts
% Used only if q_target-based detection fails.
% If Test 1 automatic detection fails, fill its range manually.
manualIdxTask1 = {
    []          % Test 1: automatic preferred
    1:387      % Test 2
    1:320      % Test 3
};

%% Summary storage
runID = [];
x0 = [];
y0 = [];
theta0 = [];
xf = [];
yf = [];
thetaf = [];
duration_s = [];
final_ep = [];
final_eth = [];
final_eth_deg = [];
comment = strings(0,1);

for k = 1:3

    fprintf('\n============================================================\n');
    fprintf('Processing Test %d\n', k);
    fprintf('============================================================\n');

    %% ============================================================
    % Load data
    %% ============================================================

    D = [];

    if isfile(exportedFiles{k})
        fprintf('Using exported file: %s\n', exportedFiles{k});
        S = load(exportedFiles{k});

        if isfield(S,'D')
            D = S.D;
        elseif isfield(S,'D_task1')
            D = S.D_task1;
        else
            error('Exported file exists, but D or D_task1 was not found inside %s.', exportedFiles{k});
        end

    elseif isfile(rawFiles{k})
        fprintf('Using raw file: %s\n', rawFiles{k});
        S = load(rawFiles{k});
        D = extractDfromRawMat(S, TsDefault);

    else
        error('Neither %s nor %s was found.', exportedFiles{k}, rawFiles{k});
    end

    %% ============================================================
    % Basic checks
    %% ============================================================

    if ~isfield(D,'t')
        error('D.t was not found for Test %d.', k);
    end

    D.t = D.t(:);

    if isfield(D,'q_target') && ~isempty(D.q_target)
        D.q_target = ensureRows(D.q_target, 3);
    else
        error('D.q_target was not found for Test %d. Cannot isolate Task 1 automatically.', k);
    end

    %% ============================================================
    % Detect Task 1 segment
    %% ============================================================

    q_target = D.q_target;

    targetDiffXY = vecnorm(q_target(:,1:2) - q_goal_task1(1:2), 2, 2);
    targetDiffTh = abs(wrapToPiVec(q_target(:,3) - q_goal_task1(3)));

    tolXY = 1e-6;
    tolTh = 1e-6;

    isTask1 = targetDiffXY < tolXY & targetDiffTh < tolTh;

    % Relax tolerance if needed
    if nnz(isTask1) < 5
        tolXY = 1e-3;
        tolTh = 1e-3;
        isTask1 = targetDiffXY < tolXY & targetDiffTh < tolTh;
    end

    % If automatic detection works, use the first continuous Task 1 interval
    if any(isTask1)
        [starts, ends] = trueSegments(isTask1);

        idxStart = starts(1);
        idxEnd   = ends(1);

        fprintf('Task 1 detected automatically from q_target.\n');

    else
        if isempty(manualIdxTask1{k})
            error(['Task 1 could not be detected automatically for Test %d, ', ...
                   'and no manual index range was provided.'], k);
        end

        idxStart = manualIdxTask1{k}(1);
        idxEnd   = manualIdxTask1{k}(end);

        fprintf('Task 1 detected using manual fallback indices.\n');
    end

    idx_task1 = idxStart:idxEnd;

    fprintf('Task 1 indices: %d:%d\n', idxStart, idxEnd);
    fprintf('Task 1 absolute time: %.3f s to %.3f s\n', D.t(idxStart), D.t(idxEnd));

    %% ============================================================
    % Slice Task 1
    %% ============================================================

    D_task1 = sliceData(D, idx_task1);

    %% ============================================================
    % Get Vicon ground truth
    %% ============================================================

    q_vicon = getViconSignal(D_task1);

    if isempty(q_vicon)
        error('No Vicon ground-truth signal found for Test %d.', k);
    end

    q_vicon = ensureRows(q_vicon, 3);

    %% ============================================================
    % Correct q0 selection
    %
    % Use first valid Vicon sample INSIDE the official initial area.
    %% ============================================================

    valid_vicon = all(isfinite(q_vicon),2) & ~all(abs(q_vicon) < 1e-12,2);

    inside_initial_area = ...
        q_vicon(:,1) >= x1 & q_vicon(:,1) <= x3 & ...
        q_vicon(:,2) >= y6 & q_vicon(:,2) <= y8;

    idxFirstValidInsideInitial = find(valid_vicon & inside_initial_area, 1, 'first');

    if isempty(idxFirstValidInsideInitial)
        warning(['No valid Vicon sample inside the official initial area was found ', ...
                 'for Test %d. Falling back to first valid Vicon sample.'], k);

        idxFirstValid = find(valid_vicon, 1, 'first');
    else
        idxFirstValid = idxFirstValidInsideInitial;
    end

    idxLastValid = find(valid_vicon, 1, 'last');

    if isempty(idxFirstValid) || isempty(idxLastValid)
        error('No valid Vicon samples found in Task 1 segment for Test %d.', k);
    end

    q0 = q_vicon(idxFirstValid,:);
    qf = q_vicon(idxLastValid,:);

    t0 = D_task1.t(idxFirstValid);
    tf = D_task1.t(idxLastValid);

    %% Save corrected q0 and metrics inside separated file
    D_task1.q0_vicon_task1 = q0;
    D_task1.qf_vicon_task1 = qf;
    D_task1.idx_q0_task1 = idxFirstValid;
    D_task1.idx_qf_task1 = idxLastValid;
    D_task1.initial_area = [x1 x3 y6 y8];

    save(outFiles{k}, 'D_task1', '-v7');
    fprintf('Saved separated Task 1 file: %s\n', outFiles{k});

    %% ============================================================
    % Compute final errors
    %% ============================================================

    ep = norm(qf(1:2) - q_goal_task1(1:2));
    eth = abs(wrapToPiScalar(q_goal_task1(3) - qf(3)));

    if ep <= posTol && eth <= thetaTol
        cmt = "Full posture threshold reached";
    elseif ep <= posTol && eth > thetaTol
        cmt = "Position threshold reached; orientation threshold not reached";
    else
        cmt = "Position threshold not reached / check run";
    end

    %% Check q0 area status
    q0_inside_initial_area = ...
        q0(1) >= x1 && q0(1) <= x3 && ...
        q0(2) >= y6 && q0(2) <= y8;

    fprintf('\nTask 1 metrics for Test %d\n', k);
    fprintf('q0 Vicon = [%.4f %.4f %.4f]^T\n', q0(1), q0(2), q0(3));
    fprintf('q0 inside official initial area = %d\n', q0_inside_initial_area);
    fprintf('qf Vicon = [%.4f %.4f %.4f]^T\n', qf(1), qf(2), qf(3));
    fprintf('duration from corrected q0 to final valid sample = %.2f s\n', tf - t0);
    fprintf('final position error = %.4f m\n', ep);
    fprintf('final orientation error = %.4f rad = %.2f deg\n', eth, rad2deg(eth));
    fprintf('comment = %s\n', cmt);

    %% ============================================================
    % Store summary
    %% ============================================================

    runID(end+1,1) = k;

    x0(end+1,1) = q0(1);
    y0(end+1,1) = q0(2);
    theta0(end+1,1) = q0(3);

    xf(end+1,1) = qf(1);
    yf(end+1,1) = qf(2);
    thetaf(end+1,1) = qf(3);

    duration_s(end+1,1) = tf - t0;
    final_ep(end+1,1) = ep;
    final_eth(end+1,1) = eth;
    final_eth_deg(end+1,1) = rad2deg(eth);
    comment(end+1,1) = cmt;
end

%% ============================================================
% Summary table
%% ============================================================

T = table( ...
    runID, ...
    x0, y0, theta0, ...
    xf, yf, thetaf, ...
    duration_s, ...
    final_ep, final_eth, final_eth_deg, ...
    comment, ...
    'VariableNames', { ...
    'Run', ...
    'x0_m', 'y0_m', 'theta0_rad', ...
    'xf_m', 'yf_m', 'thetaf_rad', ...
    'duration_s', ...
    'final_position_error_m', ...
    'final_orientation_error_rad', ...
    'final_orientation_error_deg', ...
    'success_comment'});

disp(T);

csvFile = 'task1_three_real_runs_summary.csv';
writetable(T, csvFile);
fprintf('\nSaved CSV summary: %s\n', csvFile);

%% ============================================================
% LaTeX table
%% ============================================================

latexFile = 'task1_three_real_runs_latex_table.txt';
fid = fopen(latexFile, 'w');

fprintf(fid, '\\begin{table}[H]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, '\\caption{Task 1 initial-condition tests and final real-run errors.}\n');
fprintf(fid, '\\label{tab:task1_initial_conditions}\n');
fprintf(fid, '\\small\n');
fprintf(fid, '\\begin{tabular}{c c c c c c p{3.4cm}}\n');
fprintf(fid, '\\hline\n');
fprintf(fid, 'Run & $x_0$ [m] & $y_0$ [m] & $\\theta_0$ [rad] & Final $e_p$ [m] & Final $e_\\theta$ [rad] & Success/comment \\\\\n');
fprintf(fid, '\\hline\n');

for i = 1:height(T)
    fprintf(fid, '%d & %.4f & %.4f & %.4f & %.4f & %.4f & %s \\\\\n', ...
        T.Run(i), ...
        T.x0_m(i), ...
        T.y0_m(i), ...
        T.theta0_rad(i), ...
        T.final_position_error_m(i), ...
        T.final_orientation_error_rad(i), ...
        char(T.success_comment(i)));
end

fprintf(fid, '\\hline\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n');

fclose(fid);

fprintf('Saved LaTeX table: %s\n', latexFile);

%% ============================================================
% Local functions
%% ============================================================

function D = extractDfromRawMat(S, TsDefault)

    D = struct();

    fns = fieldnames(S);

    outObj = [];

    for i = 1:numel(fns)
        val = S.(fns{i});

        if isa(val, 'Simulink.SimulationOutput')
            outObj = val;
            fprintf('Found Simulink.SimulationOutput variable: %s\n', fns{i});
            break;
        end
    end

    if isempty(outObj)
        error('No Simulink.SimulationOutput object was found in the raw MAT file.');
    end

    try
        disp('Available variables inside SimulationOutput:');
        disp(outObj.who);
    catch
    end

    D.t = getOutSignal(outObj, {'t','tout','time'});

    D.q_target = getOutSignal(outObj, {'q_target','q_goal','q_ref'});
    D.q_est    = getOutSignal(outObj, {'q_est','q_hat','q_localizer'});

    D.Y_viscon_ground_trith = getOutSignal(outObj, { ...
        'Y_viscon_ground_trith', ...
        'Y_vicon_ground_truth', ...
        'Y_viscon_ground_truth', ...
        'Y_vicon', ...
        'Y_viscon', ...
        'q_vicon', ...
        'vicon'});

    D.omega_wheels = getOutSignal(outObj, { ...
        'omega_wheels', ...
        'omega_wheel', ...
        'w_cmd', ...
        'wheel_cmd', ...
        'cmd_wheels'});

    D.bot_wheelspeed = getOutSignal(outObj, { ...
        'bot_wheelspeed', ...
        'wheelSpeeds', ...
        'w_meas', ...
        'wheel_meas', ...
        'measured_wheel_speed'});

    if isempty(D.t)
        D.t = getOutTime(outObj, { ...
            'Y_viscon_ground_trith', ...
            'Y_vicon_ground_truth', ...
            'Y_viscon_ground_truth', ...
            'q_target', ...
            'q_est'});
    end

    if isempty(D.t)
        n = [];

        if ~isempty(D.q_target)
            n = size(ensureRows(D.q_target,3),1);
        elseif ~isempty(D.Y_viscon_ground_trith)
            n = size(ensureRows(D.Y_viscon_ground_trith,3),1);
        elseif ~isempty(D.q_est)
            n = size(ensureRows(D.q_est,3),1);
        end

        if isempty(n)
            error('Could not determine signal length to reconstruct time.');
        end

        D.t = (0:n-1)' * TsDefault;
        warning('Time vector not found. Reconstructed using Ts=0.04 s.');
    end

    D.t = D.t(:);

    if ~isempty(D.q_target), D.q_target = ensureRows(D.q_target,3); end
    if ~isempty(D.q_est), D.q_est = ensureRows(D.q_est,3); end
    if ~isempty(D.Y_viscon_ground_trith), D.Y_viscon_ground_trith = ensureRows(D.Y_viscon_ground_trith,3); end
    if ~isempty(D.omega_wheels), D.omega_wheels = ensureRows(D.omega_wheels,2); end
    if ~isempty(D.bot_wheelspeed), D.bot_wheelspeed = ensureRows(D.bot_wheelspeed,2); end

    nMin = numel(D.t);

    names = fieldnames(D);

    for i = 1:numel(names)
        x = D.(names{i});

        if isnumeric(x) && size(x,1) > 1
            nMin = min(nMin, size(x,1));
        end
    end

    for i = 1:numel(names)
        x = D.(names{i});

        if isnumeric(x) && size(x,1) >= nMin
            D.(names{i}) = x(1:nMin,:);
        end
    end
end

function X = getOutSignal(outObj, names)

    X = [];

    for i = 1:numel(names)
        try
            val = outObj.get(names{i});
            X = signalToArray(val);

            if ~isempty(X)
                return;
            end
        catch
        end
    end

    try
        logsout = outObj.get('logsout');

        if isa(logsout,'Simulink.SimulationData.Dataset')
            for i = 1:numel(names)
                try
                    el = logsout.get(names{i});
                    X = signalToArray(el);

                    if ~isempty(X)
                        return;
                    end
                catch
                end
            end
        end
    catch
    end

    try
        yout = outObj.get('yout');

        if isa(yout,'Simulink.SimulationData.Dataset')
            for i = 1:numel(names)
                try
                    el = yout.get(names{i});
                    X = signalToArray(el);

                    if ~isempty(X)
                        return;
                    end
                catch
                end
            end
        end
    catch
    end
end

function t = getOutTime(outObj, names)

    t = [];

    for i = 1:numel(names)
        try
            val = outObj.get(names{i});

            if isa(val,'timeseries')
                t = val.Time;
                return;
            elseif isa(val,'Simulink.SimulationData.Signal')
                if isa(val.Values,'timeseries')
                    t = val.Values.Time;
                    return;
                end
            end
        catch
        end
    end

    try
        t = outObj.tout;
    catch
    end
end

function A = signalToArray(val)

    A = [];

    if isempty(val)
        return;
    end

    if isa(val,'timeseries')
        A = val.Data;

    elseif isa(val,'Simulink.SimulationData.Signal')
        A = signalToArray(val.Values);

    elseif isa(val,'Simulink.SimulationData.Dataset')
        return;

    elseif isstruct(val) && isfield(val,'signals') && isfield(val.signals,'values')
        A = val.signals.values;

    elseif isstruct(val) && isfield(val,'Data')
        A = val.Data;

    else
        A = val;
    end

    try
        A = squeeze(double(A));
    catch
        A = squeeze(A);
    end
end

function Dout = sliceData(Din, idx)

    fields = fieldnames(Din);
    Dout = struct();

    for i = 1:length(fields)

        name = fields{i};
        x = Din.(name);

        if isnumeric(x) && size(x,1) >= max(idx)
            Dout.(name) = x(idx,:);
        else
            Dout.(name) = x;
        end
    end

    Dout.t_abs = Din.t(idx);
    Dout.t = Din.t(idx) - Din.t(idx(1));
end

function q_vicon = getViconSignal(D)

    q_vicon = [];

    possibleNames = { ...
        'Y_viscon_ground_trith', ...
        'Y_vicon_ground_truth', ...
        'Y_viscon_ground_truth', ...
        'Y_vicon', ...
        'Y_viscon', ...
        'vicon', ...
        'q_vicon'};

    for i = 1:numel(possibleNames)
        name = possibleNames{i};

        if isfield(D, name)
            q_vicon = D.(name);
            return;
        end
    end
end

function A = ensureRows(A, nCols)

    if isempty(A)
        return;
    end

    A = squeeze(A);

    if isvector(A)
        if numel(A) == nCols
            A = reshape(A, 1, nCols);
        else
            A = A(:);
        end
    else
        if size(A,2) ~= nCols && size(A,1) == nCols
            A = A.';
        end
    end
end

function [starts, ends] = trueSegments(mask)

    mask = mask(:);
    d = diff([false; mask; false]);

    starts = find(d == 1);
    ends   = find(d == -1) - 1;
end

function a = wrapToPiScalar(a)

    a = atan2(sin(a), cos(a));
end

function a = wrapToPiVec(a)

    a = atan2(sin(a), cos(a));
end
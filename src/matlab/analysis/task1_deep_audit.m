%% TASK 1 DEEP AUDIT FROM OPEN SIMULINK + DATA
% Save this file as: task1_deep_audit_from_open_simulink.m
% Then run it while the Task 1 Simulink model is open.

clc;
format long g;

auditDir = fullfile(pwd, 'task1_audit');
if ~exist(auditDir, 'dir')
    mkdir(auditDir);
end

diary off;
diaryFile = fullfile(auditDir, 'task1_audit_report.txt');
if exist(diaryFile, 'file')
    delete(diaryFile);
end
diary(diaryFile);

fprintf('\n============================================================\n');
fprintf('TASK 1 DEEP AUDIT REPORT\n');
fprintf('Generated at: %s\n', datestr(now));
fprintf('Current folder: %s\n', pwd);
fprintf('============================================================\n\n');

%% ------------------------------------------------------------
% 1) Detect currently open Simulink model
%% ------------------------------------------------------------
fprintf('1) OPEN SIMULINK MODELS\n');
fprintf('------------------------------------------------------------\n');

try
    loadedModels = find_system('Type', 'block_diagram');
catch
    loadedModels = {};
end

loadedModels = setdiff(loadedModels, {'simulink','simulink_extras'});

if isempty(loadedModels)
    fprintf('ERROR: No Simulink model is open.\n');
    fprintf('Open the Task 1 model and run this script again.\n');
    diary off;
    return;
end

try
    currentModel = bdroot(gcs);
catch
    currentModel = loadedModels{1};
end

if isempty(currentModel) || ~ismember(currentModel, loadedModels)
    currentModel = loadedModels{1};
end

fprintf('Loaded models:\n');
for i = 1:numel(loadedModels)
    fprintf('  %d) %s\n', i, loadedModels{i});
end
fprintf('\nSelected model for audit: %s\n\n', currentModel);

%% ------------------------------------------------------------
% 2) Base workspace variables
%% ------------------------------------------------------------
fprintf('\n2) BASE WORKSPACE PARAMETERS\n');
fprintf('------------------------------------------------------------\n');

paramNames = { ...
    'Ts','r','d', ...
    'k1','k2','k_theta', ...
    'k_x','k_y', ...
    'v_max','omega_max', ...
    'v_limit','omega_limit', ...
    'omega_M','dist_threshold','theta_threshold', ...
    'posTol','thetaTol', ...
    'q_init','q_goal1','q_goal2'};

for i = 1:numel(paramNames)
    printBaseVar(paramNames{i});
end

%% ------------------------------------------------------------
% 3) Extract MATLAB Function / Stateflow chart code from open model
%% ------------------------------------------------------------
fprintf('\n3) MATLAB FUNCTION BLOCKS / STATEFLOW CHARTS\n');
fprintf('------------------------------------------------------------\n');

controllerScriptsFile = fullfile(auditDir, 'task1_controller_and_function_blocks.txt');
fidScripts = fopen(controllerScriptsFile, 'w');

controller_v_limit = NaN;
controller_omega_limit = NaN;
controller_rho_switch = NaN;
controller_theta_stop = NaN;

try
    rt = sfroot;
    charts = rt.find('-isa','Stateflow.EMChart');

    fprintf('Found %d MATLAB Function charts in open Simulink session.\n\n', numel(charts));
    fprintf(fidScripts, 'TASK 1 MATLAB FUNCTION BLOCKS\n');
    fprintf(fidScripts, 'Model selected: %s\n\n', currentModel);

    for i = 1:numel(charts)
        ch = charts(i);

        try
            chPath = ch.Path;
        catch
            chPath = '';
        end

        try
            chName = ch.Name;
        catch
            chName = '';
        end

        if ~contains(chPath, currentModel)
            continue;
        end

        script = ch.Script;
        lowerID = lower([chPath ' ' chName]);

        fprintf('\n--- Chart %d ---\n', i);
        fprintf('Name: %s\n', chName);
        fprintf('Path: %s\n', chPath);

        fprintf(fidScripts, '\n\n============================================================\n');
        fprintf(fidScripts, 'Name: %s\n', chName);
        fprintf(fidScripts, 'Path: %s\n', chPath);
        fprintf(fidScripts, '============================================================\n');
        fprintf(fidScripts, '%s\n', script);

        if contains(lowerID, 'controller')
            fprintf('\n>>> CONTROLLER BLOCK FOUND: %s\n', chPath);

            vTok = regexp(script, 'v_limit\s*=\s*([^;]+);', 'tokens', 'once');
            omTok = regexp(script, 'omega_limit\s*=\s*([^;]+);', 'tokens', 'once');
            rhoTok = regexp(script, 'rho\s*<\s*([0-9.eE+-]+)', 'tokens', 'once');
            thTok = regexp(script, 'abs\s*\(\s*e_theta\s*\)\s*<\s*([0-9.eE+-]+)', 'tokens', 'once');

            if ~isempty(vTok)
                controller_v_limit = str2double(strtrim(vTok{1}));
                fprintf('Controller v_limit extracted      = %s\n', strtrim(vTok{1}));
            else
                fprintf('Controller v_limit extracted      = NOT FOUND\n');
            end

            if ~isempty(omTok)
                controller_omega_limit = str2double(strtrim(omTok{1}));
                fprintf('Controller omega_limit extracted  = %s\n', strtrim(omTok{1}));
            else
                fprintf('Controller omega_limit extracted  = NOT FOUND\n');
            end

            if ~isempty(rhoTok)
                controller_rho_switch = str2double(strtrim(rhoTok{1}));
                fprintf('Controller rho switch threshold   = %s\n', strtrim(rhoTok{1}));
            else
                fprintf('Controller rho switch threshold   = NOT FOUND\n');
            end

            if ~isempty(thTok)
                controller_theta_stop = str2double(strtrim(thTok{1}));
                fprintf('Controller theta stop threshold   = %s\n', strtrim(thTok{1}));
            else
                fprintf('Controller theta stop threshold   = NOT FOUND\n');
            end
        end
    end

catch ME
    fprintf('WARNING: Could not extract Stateflow/MATLAB Function scripts.\n');
    fprintf('Reason: %s\n', ME.message);
end

fclose(fidScripts);
fprintf('\nAll extracted function-block scripts saved in:\n%s\n', controllerScriptsFile);

%% ------------------------------------------------------------
% 4) Find Saturation blocks in the selected Simulink model
%% ------------------------------------------------------------
fprintf('\n4) SATURATION BLOCKS IN OPEN MODEL\n');
fprintf('------------------------------------------------------------\n');

try
    satBlocks = find_system(currentModel, ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'BlockType','Saturate');

    if isempty(satBlocks)
        fprintf('No Simulink Saturate blocks found.\n');
    else
        for i = 1:numel(satBlocks)
            blk = satBlocks{i};
            fprintf('\nSaturate block %d:\n', i);
            fprintf('Path: %s\n', blk);
            try
                fprintf('UpperLimit: %s\n', get_param(blk,'UpperLimit'));
            catch
                fprintf('UpperLimit: could not read\n');
            end
            try
                fprintf('LowerLimit: %s\n', get_param(blk,'LowerLimit'));
            catch
                fprintf('LowerLimit: could not read\n');
            end
        end
    end
catch ME
    fprintf('WARNING: Could not inspect Saturate blocks.\n');
    fprintf('Reason: %s\n', ME.message);
end

%% ------------------------------------------------------------
% 5) Load real and simulation data
%% ------------------------------------------------------------
fprintf('\n5) LOAD TASK 1 DATA\n');
fprintf('------------------------------------------------------------\n');

D = loadStructSmart('D_task1', 'test3_task1_all_outputs.mat');
Dsim = loadStructSmart('Dsim_task1_simulation', 'exported_task1_simulation_extended_test3.mat');

if isempty(D)
    fprintf('ERROR: Could not load D_task1.\n');
    fprintf('Put test3_task1_all_outputs.mat in the current folder or load D_task1 in workspace.\n');
    diary off;
    return;
end

if isempty(Dsim)
    fprintf('ERROR: Could not load Dsim_task1_simulation.\n');
    fprintf('Put exported_task1_simulation_extended_test3.mat in current folder or load Dsim_task1_simulation in workspace.\n');
    diary off;
    return;
end

fprintf('D_task1 fields:\n');
disp(fieldnames(D));

fprintf('Dsim_task1_simulation fields:\n');
disp(fieldnames(Dsim));

%% ------------------------------------------------------------
% 6) Extract signals
%% ------------------------------------------------------------
fprintf('\n6) SIGNAL EXTRACTION\n');
fprintf('------------------------------------------------------------\n');

tReal = sig2vec(getFieldAny(D, {'t','tout','time'}));
tSim  = sig2vec(getFieldAny(Dsim, {'t','tout','time'}));

qVicon = asRows(getFieldAny(D, { ...
    'Y_viscon_ground_trith', ...
    'Y_vicon_ground_truth', ...
    'Y_viscon_ground_truth', ...
    'Y_vicon', ...
    'Y_viscon'}), 3);

qViconRaw = asRows(getFieldAny(D, {'Y_viscon','Y_vicon'}), 3);
qLoc      = asRows(getFieldAny(D, {'q_est','q_localizer','q_hat'}), 3);
qOdomLog  = asRows(getFieldAny(D, {'q_est_imu','q_odom','q_odometry','q_wheel_odom'}), 3);
qTarget   = asRows(getFieldAny(D, {'q_target','q_ref','q_goal'}), 3);

qSim      = asRows(getFieldAny(Dsim, {'q_actual','q','q_true'}), 3);
qSimLoc   = asRows(getFieldAny(Dsim, {'q_est','q_localizer','q_hat'}), 3);
qSimTarget = asRows(getFieldAny(Dsim, {'q_target','q_ref','q_goal'}), 3);

vReal     = sig2vec(getFieldAny(D, {'v','out_v'}));
omegaReal = sig2vec(getFieldAny(D, {'omega','out_omega'}));
wCmdReal  = asRows(getFieldAny(D, {'omega_wheels','w_cmd','wheel_cmd'}), 2);
wMeasReal = asRows(getFieldAny(D, {'bot_wheelspeed','w_meas','wheelSpeeds','measured_wheel_speed'}), 2);

vSim       = sig2vec(getFieldAny(Dsim, {'v','out_v'}));
omegaSim   = sig2vec(getFieldAny(Dsim, {'omega','out_omega'}));
wCmdSim    = asRows(getFieldAny(Dsim, {'omega_wheels','w_cmd','wheel_cmd'}), 2);

if isempty(qTarget)
    qGoal = sig2vec(getFieldAny(D, {'q_goal'}));
    qGoal = qGoal(:).';
else
    qGoal = qTarget(1,:);
end

fprintf('Extracted sizes:\n');
fprintf('tReal      : %s\n', mat2str(size(tReal)));
fprintf('qVicon     : %s\n', mat2str(size(qVicon)));
fprintf('qLoc       : %s\n', mat2str(size(qLoc)));
fprintf('qOdomLog   : %s\n', mat2str(size(qOdomLog)));
fprintf('qTarget    : %s\n', mat2str(size(qTarget)));
fprintf('wCmdReal   : %s\n', mat2str(size(wCmdReal)));
fprintf('wMeasReal  : %s\n', mat2str(size(wMeasReal)));
fprintf('qSim       : %s\n', mat2str(size(qSim)));
fprintf('wCmdSim    : %s\n', mat2str(size(wCmdSim)));

%% ------------------------------------------------------------
% 7) Geometry and threshold parameters
%% ------------------------------------------------------------
fprintf('\n7) GEOMETRY AND THRESHOLDS\n');
fprintf('------------------------------------------------------------\n');

r = getBaseNumber({'r','wheel_radius'});
d = getBaseNumber({'d','wheel_distance','wheel_base'});

if isnan(r)
    r = 0.03;
    fprintf('WARNING: r not found in base workspace. Using r = 0.03 from uploaded model/script for diagnostic.\n');
end
if isnan(d)
    d = 0.165;
    fprintf('WARNING: d not found in base workspace. Using d = 0.165 from uploaded model/script for diagnostic.\n');
end

posTol = getBaseNumber({'dist_threshold','posTol','position_threshold','eps_pos'});
thetaTol = getBaseNumber({'theta_threshold','thetaTol','eps_theta'});

if isnan(posTol)
    posTol = 0.05;
    fprintf('WARNING: position threshold variable not found. Using 0.05 only as diagnostic/report tolerance; confirm before final report.\n');
end

if isnan(thetaTol)
    if ~isnan(controller_theta_stop)
        thetaTol = controller_theta_stop;
        fprintf('thetaTol taken from Controller block abs(e_theta)<... = %.6f\n', thetaTol);
    else
        thetaTol = 0.05;
        fprintf('WARNING: theta threshold variable not found. Using 0.05 only as diagnostic/report tolerance; confirm before final report.\n');
    end
end

fprintf('r        = %.6f m\n', r);
fprintf('d        = %.6f m\n', d);
fprintf('posTol   = %.6f m\n', posTol);
fprintf('thetaTol = %.6f rad\n', thetaTol);

%% ------------------------------------------------------------
% 8) Compute metrics
%% ------------------------------------------------------------
fprintf('\n8) NUMERICAL METRICS FROM DATA\n');
fprintf('------------------------------------------------------------\n');

wrapPi = @(a) atan2(sin(a), cos(a));

q0Vicon = qVicon(1,:);
q0Sim   = qSim(1,:);

fprintf('q0 Vicon = [%.6f %.6f %.6f]^T\n', q0Vicon);
fprintf('q0 Sim   = [%.6f %.6f %.6f]^T\n', q0Sim);
fprintf('qGoal    = [%.6f %.6f %.6f]^T\n', qGoal);

q0Difference = norm(q0Vicon - q0Sim);
fprintf('||q0 Vicon - q0 Sim|| = %.12f\n', q0Difference);

realDuration = tReal(end);
simDuration  = tSim(end);

fprintf('Real duration = %.6f s\n', realDuration);
fprintf('Simulation duration = %.6f s\n', simDuration);

ePosVicon = vecnorm(qVicon(:,1:2) - qGoal(1:2), 2, 2);
eThVicon  = abs(wrapPi(qGoal(3) - qVicon(:,3)));

ePosSim = vecnorm(qSim(:,1:2) - qGoal(1:2), 2, 2);
eThSim  = abs(wrapPi(qGoal(3) - qSim(:,3)));

idxRealEndSim = find(tSim <= realDuration, 1, 'last');
if isempty(idxRealEndSim)
    [~, idxRealEndSim] = min(abs(tSim - realDuration));
end

idxConvSim = find(ePosSim <= posTol & eThSim <= thetaTol, 1, 'first');

finalViconPosErr = ePosVicon(end);
finalViconThetaErr = eThVicon(end);

simAtRealEndPosErr = ePosSim(idxRealEndSim);
simAtRealEndThetaErr = eThSim(idxRealEndSim);

if isempty(idxConvSim)
    simConvTime = NaN;
    simFinalPosErr = ePosSim(end);
    simFinalThetaErr = eThSim(end);
else
    simConvTime = tSim(idxConvSim);
    simFinalPosErr = ePosSim(idxConvSim);
    simFinalThetaErr = eThSim(idxConvSim);
end

fprintf('\nFinal Vicon position error = %.6f m\n', finalViconPosErr);
fprintf('Final Vicon theta error    = %.6f rad = %.3f deg\n', ...
    finalViconThetaErr, rad2deg(finalViconThetaErr));

fprintf('\nSimulation at real end:\n');
fprintf('position error = %.6f m\n', simAtRealEndPosErr);
fprintf('theta error    = %.6f rad = %.3f deg\n', ...
    simAtRealEndThetaErr, rad2deg(simAtRealEndThetaErr));

fprintf('\nSimulation convergence with selected thresholds:\n');
if isempty(idxConvSim)
    fprintf('Simulation did NOT reach both thresholds within tSim.\n');
else
    fprintf('t = %.6f s\n', simConvTime);
    fprintf('position error = %.6f m\n', simFinalPosErr);
    fprintf('theta error    = %.6f rad = %.3f deg\n', ...
        simFinalThetaErr, rad2deg(simFinalThetaErr));
end

fprintf('\nThreshold flags:\n');
fprintf('Real final position threshold reached: %d\n', finalViconPosErr <= posTol);
fprintf('Real final theta threshold reached   : %d\n', finalViconThetaErr <= thetaTol);
fprintf('Simulation both thresholds reached   : %d\n', ~isempty(idxConvSim));

%% ------------------------------------------------------------
% 9) Localizer and odometry diagnostics
%% ------------------------------------------------------------
fprintf('\n9) LOCALIZER / WHEEL ODOMETRY DIAGNOSTICS\n');
fprintf('------------------------------------------------------------\n');

Nloc = min(size(qVicon,1), size(qLoc,1));
locXYErr = vecnorm(qLoc(1:Nloc,1:2) - qVicon(1:Nloc,1:2), 2, 2);
locThErr = abs(wrapPi(qVicon(1:Nloc,3) - qLoc(1:Nloc,3)));

fprintf('Localizer vs Vicon:\n');
fprintf('Mean XY error = %.6f m\n', mean(locXYErr,'omitnan'));
fprintf('Max XY error  = %.6f m\n', max(locXYErr,[],'omitnan'));
fprintf('Mean theta error = %.6f rad = %.3f deg\n', ...
    mean(locThErr,'omitnan'), rad2deg(mean(locThErr,'omitnan')));
fprintf('Max theta error  = %.6f rad = %.3f deg\n', ...
    max(locThErr,[],'omitnan'), rad2deg(max(locThErr,[],'omitnan')));

if ~isempty(qOdomLog)
    Nod = min(size(qVicon,1), size(qOdomLog,1));
    odomLogXYErr = vecnorm(qOdomLog(1:Nod,1:2) - qVicon(1:Nod,1:2), 2, 2);
    odomLogThErr = abs(wrapPi(qVicon(1:Nod,3) - qOdomLog(1:Nod,3)));

    fprintf('\nLogged q_est_imu / odometry block vs Vicon:\n');
    fprintf('Mean XY error = %.6f m\n', mean(odomLogXYErr,'omitnan'));
    fprintf('Max XY error  = %.6f m\n', max(odomLogXYErr,[],'omitnan'));
    fprintf('Mean theta error = %.6f rad = %.3f deg\n', ...
        mean(odomLogThErr,'omitnan'), rad2deg(mean(odomLogThErr,'omitnan')));
end

if ~isempty(wMeasReal)
    qOdomNoSwap = reconstructWheelOdom(tReal, wMeasReal, q0Vicon, r, d);
    qOdomSwap   = reconstructWheelOdom(tReal, wMeasReal(:,[2 1]), q0Vicon, r, d);

    errNoSwap = vecnorm(qOdomNoSwap(:,1:2) - qVicon(:,1:2), 2, 2);
    errSwap   = vecnorm(qOdomSwap(:,1:2)   - qVicon(:,1:2), 2, 2);

    thNoSwap = abs(wrapPi(qVicon(:,3) - qOdomNoSwap(:,3)));
    thSwap   = abs(wrapPi(qVicon(:,3) - qOdomSwap(:,3)));

    fprintf('\nWheel odometry reconstructed from measured wheel speeds:\n');
    fprintf('NO SWAP  mean XY error = %.6f m | max XY error = %.6f m | mean theta error = %.6f rad\n', ...
        mean(errNoSwap,'omitnan'), max(errNoSwap,[],'omitnan'), mean(thNoSwap,'omitnan'));
    fprintf('SWAPPED  mean XY error = %.6f m | max XY error = %.6f m | mean theta error = %.6f rad\n', ...
        mean(errSwap,'omitnan'), max(errSwap,[],'omitnan'), mean(thSwap,'omitnan'));

    if mean(errSwap,'omitnan') < mean(errNoSwap,'omitnan')
        fprintf('DIAGNOSIS: measured wheel-speed columns look SWAPPED for odometry reconstruction.\n');
    else
        fprintf('DIAGNOSIS: measured wheel-speed columns do NOT look swapped for odometry reconstruction.\n');
    end
end

%% ------------------------------------------------------------
% 10) Wheel speed and saturation diagnosis
%% ------------------------------------------------------------
fprintf('\n10) WHEEL SPEED / SATURATION DIAGNOSIS\n');
fprintf('------------------------------------------------------------\n');

maxRealV = max(abs(vReal), [], 'omitnan');
maxRealOmegaLogged = max(abs(omegaReal), [], 'omitnan');
maxCmdWheelReal = max(abs(wCmdReal), [], 'all', 'omitnan');
maxMeasWheelReal = max(abs(wMeasReal), [], 'all', 'omitnan');

maxSimV = max(abs(vSim), [], 'omitnan');
maxSimOmega = max(abs(omegaSim), [], 'omitnan');
maxCmdWheelSim = max(abs(wCmdSim), [], 'all', 'omitnan');

fprintf('Real max |v logged|       = %.6f m/s\n', maxRealV);
fprintf('Real max |omega logged|   = %.6f rad/s\n', maxRealOmegaLogged);
fprintf('Real max |wheel command|  = %.6f rad/s\n', maxCmdWheelReal);
fprintf('Real max |wheel measured| = %.6f rad/s\n', maxMeasWheelReal);

fprintf('\nSimulation max |v|        = %.6f m/s\n', maxSimV);
fprintf('Simulation max |omega|    = %.6f rad/s\n', maxSimOmega);
fprintf('Simulation max |wheel cmd|= %.6f rad/s\n', maxCmdWheelSim);

if ~isempty(vReal) && ~isempty(omegaReal)
    wFromLoggedVO = [vReal(:)/r - d*omegaReal(:)/(2*r), ...
                     vReal(:)/r + d*omegaReal(:)/(2*r)];

    maxWheelFromLoggedVO = max(abs(wFromLoggedVO), [], 'all', 'omitnan');

    fprintf('\nIf logged v/omega are converted directly to wheel speeds:\n');
    fprintf('Max |wheel speed from logged v/omega| = %.6f rad/s\n', maxWheelFromLoggedVO);

    if ~isempty(wCmdReal)
        wheelDiff = wFromLoggedVO(1:min(end,size(wCmdReal,1)),:) - ...
                    wCmdReal(1:min(end,size(wFromLoggedVO,1)),:);
        fprintf('Mean |wheel_from_logged_vomega - logged_wheel_command| = %.6f rad/s\n', ...
            mean(abs(wheelDiff), 'all', 'omitnan'));
        fprintf('Max  |wheel_from_logged_vomega - logged_wheel_command| = %.6f rad/s\n', ...
            max(abs(wheelDiff), [], 'all', 'omitnan'));
    end

    if maxWheelFromLoggedVO > maxCmdWheelReal + 1e-3
        fprintf(['DIAGNOSIS: logged v/omega appear to be BEFORE wheel-speed saturation, ' ...
                 'because direct conversion gives wheel speeds above the logged saturated wheel command.\n']);
    else
        fprintf('DIAGNOSIS: logged v/omega are consistent with logged wheel commands after saturation.\n');
    end
end

if ~isempty(wCmdReal)
    uFromWCmdReal = [r/2*(wCmdReal(:,1)+wCmdReal(:,2)), ...
                     r/d*(wCmdReal(:,2)-wCmdReal(:,1))];

    fprintf('\nBody input implied by REAL logged wheel commands:\n');
    fprintf('Max |v implied by wheel command|     = %.6f m/s\n', max(abs(uFromWCmdReal(:,1)),[],'omitnan'));
    fprintf('Max |omega implied by wheel command| = %.6f rad/s\n', max(abs(uFromWCmdReal(:,2)),[],'omitnan'));
end

if ~isempty(wCmdSim)
    uFromWCmdSim = [r/2*(wCmdSim(:,1)+wCmdSim(:,2)), ...
                    r/d*(wCmdSim(:,2)-wCmdSim(:,1))];

    fprintf('\nBody input implied by SIM logged wheel commands:\n');
    fprintf('Max |v implied by sim wheel command|     = %.6f m/s\n', max(abs(uFromWCmdSim(:,1)),[],'omitnan'));
    fprintf('Max |omega implied by sim wheel command| = %.6f rad/s\n', max(abs(uFromWCmdSim(:,2)),[],'omitnan'));
end

%% ------------------------------------------------------------
% 11) Save compact metrics table
%% ------------------------------------------------------------
fprintf('\n11) SAVE METRICS TABLE\n');
fprintf('------------------------------------------------------------\n');

MN = strings(0,1);
MV = [];
MU = strings(0,1);
MT = strings(0,1);

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q0_vicon_x', q0Vicon(1), 'm', 'Initial Vicon pose');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q0_vicon_y', q0Vicon(2), 'm', 'Initial Vicon pose');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q0_vicon_theta', q0Vicon(3), 'rad', 'Initial Vicon pose');

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q0_sim_x', q0Sim(1), 'm', 'Simulation initial pose');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q0_sim_y', q0Sim(2), 'm', 'Simulation initial pose');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q0_sim_theta', q0Sim(3), 'rad', 'Simulation initial pose');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q0_difference_norm', q0Difference, '-', 'Norm of q0_vicon - q0_sim');

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q_goal_x', qGoal(1), 'm', 'Task 1 goal');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q_goal_y', qGoal(2), 'm', 'Task 1 goal');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'q_goal_theta', qGoal(3), 'rad', 'Task 1 goal');

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'real_duration', realDuration, 's', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_duration', simDuration, 's', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_convergence_time', simConvTime, 's', 'Both position and theta thresholds');

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'final_vicon_position_error', finalViconPosErr, 'm', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'final_vicon_theta_error', finalViconThetaErr, 'rad', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'final_vicon_theta_error_deg', rad2deg(finalViconThetaErr), 'deg', '');

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_at_real_end_position_error', simAtRealEndPosErr, 'm', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_at_real_end_theta_error', simAtRealEndThetaErr, 'rad', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_at_real_end_theta_error_deg', rad2deg(simAtRealEndThetaErr), 'deg', '');

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_converged_position_error', simFinalPosErr, 'm', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_converged_theta_error', simFinalThetaErr, 'rad', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'sim_converged_theta_error_deg', rad2deg(simFinalThetaErr), 'deg', '');

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'mean_localizer_vicon_xy_error', mean(locXYErr,'omitnan'), 'm', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_localizer_vicon_xy_error', max(locXYErr,[],'omitnan'), 'm', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'mean_localizer_vicon_theta_error', mean(locThErr,'omitnan'), 'rad', '');

if exist('errSwap','var')
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'raw_wheel_odom_mean_xy_error_swapped', mean(errSwap,'omitnan'), 'm', 'Reconstructed from measured wheel speeds, swapped columns');
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'raw_wheel_odom_max_xy_error_swapped', max(errSwap,[],'omitnan'), 'm', 'Reconstructed from measured wheel speeds, swapped columns');
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'raw_wheel_odom_mean_theta_error_swapped', mean(thSwap,'omitnan'), 'rad', 'Reconstructed from measured wheel speeds, swapped columns');
end

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_real_v_logged', maxRealV, 'm/s', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_real_omega_logged', maxRealOmegaLogged, 'rad/s', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_real_wheel_command', maxCmdWheelReal, 'rad/s', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_real_wheel_measured', maxMeasWheelReal, 'rad/s', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_sim_v', maxSimV, 'm/s', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_sim_omega', maxSimOmega, 'rad/s', '');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_sim_wheel_command', maxCmdWheelSim, 'rad/s', '');

if exist('maxWheelFromLoggedVO','var')
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_wheel_from_logged_v_omega', maxWheelFromLoggedVO, 'rad/s', 'Diagnostic: before/after wheel saturation');
end

if exist('uFromWCmdReal','var')
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_v_implied_by_real_wheel_command', max(abs(uFromWCmdReal(:,1)),[],'omitnan'), 'm/s', 'After wheel saturation');
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_omega_implied_by_real_wheel_command', max(abs(uFromWCmdReal(:,2)),[],'omitnan'), 'rad/s', 'After wheel saturation');
end

if exist('uFromWCmdSim','var')
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_v_implied_by_sim_wheel_command', max(abs(uFromWCmdSim(:,1)),[],'omitnan'), 'm/s', 'Simulation wheel command');
    [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'max_omega_implied_by_sim_wheel_command', max(abs(uFromWCmdSim(:,2)),[],'omitnan'), 'rad/s', 'Simulation wheel command');
end

[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'controller_v_limit_from_open_model', controller_v_limit, 'm/s', 'Extracted from Controller block');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'controller_omega_limit_from_open_model', controller_omega_limit, 'rad/s', 'Extracted from Controller block');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'controller_rho_switch_from_open_model', controller_rho_switch, 'm', 'Extracted from Controller block');
[MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,'controller_theta_stop_from_open_model', controller_theta_stop, 'rad', 'Extracted from Controller block');

Tmetrics = table(MN, MV, MU, MT, ...
    'VariableNames', {'Quantity','Value','Unit','Note'});

metricsFile = fullfile(auditDir, 'task1_metrics_from_data.csv');
writetable(Tmetrics, metricsFile);
disp(Tmetrics);

fprintf('\nMetrics table saved in:\n%s\n', metricsFile);

fprintf('\n============================================================\n');
fprintf('AUDIT COMPLETE.\n');
fprintf('Please send me:\n');
fprintf('1) %s\n', diaryFile);
fprintf('2) %s\n', metricsFile);
fprintf('3) %s\n', controllerScriptsFile);
fprintf('============================================================\n');

diary off;

%% ============================================================
% Local helper functions
%% ============================================================

function printBaseVar(name)
    try
        existsFlag = evalin('base', sprintf('exist(''%s'',''var'')', name));
        if existsFlag
            val = evalin('base', name);
            if isa(val, 'Simulink.Parameter')
                val = val.Value;
            end
            if isnumeric(val)
                if numel(val) <= 20
                    fprintf('%s = %s\n', name, mat2str(double(val), 8));
                else
                    fprintf('%s = numeric array, size %s\n', name, mat2str(size(val)));
                end
            else
                fprintf('%s exists, class = %s\n', name, class(val));
            end
        else
            fprintf('%s = NOT FOUND in base workspace\n', name);
        end
    catch ME
        fprintf('%s = could not read (%s)\n', name, ME.message);
    end
end

function val = getBaseNumber(names)
    val = NaN;
    for i = 1:numel(names)
        name = names{i};
        try
            existsFlag = evalin('base', sprintf('exist(''%s'',''var'')', name));
            if existsFlag
                tmp = evalin('base', name);
                if isa(tmp, 'Simulink.Parameter')
                    tmp = tmp.Value;
                end
                if isnumeric(tmp) && isscalar(tmp)
                    val = double(tmp);
                    return;
                end
            end
        catch
        end
    end
end

function S = loadStructSmart(varName, fileName)
    S = [];

    try
        existsFlag = evalin('base', sprintf('exist(''%s'',''var'')', varName));
        if existsFlag
            S = evalin('base', varName);
            fprintf('Loaded %s from base workspace.\n', varName);
            return;
        end
    catch
    end

    if exist(fileName, 'file')
        M = load(fileName);
        if isfield(M, varName)
            S = M.(varName);
            fprintf('Loaded %s from file: %s\n', varName, fileName);
            return;
        else
            f = fieldnames(M);
            for k = 1:numel(f)
                if isstruct(M.(f{k}))
                    S = M.(f{k});
                    fprintf('WARNING: %s not found. Loaded struct %s from file %s instead.\n', ...
                        varName, f{k}, fileName);
                    return;
                end
            end
        end
    end

    fprintf('Could not automatically find %s or file %s.\n', varName, fileName);
end

function x = getFieldAny(S, candidates)
    x = [];
    if isempty(S) || ~isstruct(S)
        return;
    end

    for i = 1:numel(candidates)
        name = candidates{i};
        if isfield(S, name)
            x = S.(name);
            return;
        end
    end
end

function A = sig2mat(x)
    A = [];
    if isempty(x)
        return;
    end

    if isa(x, 'timeseries')
        A = x.Data;
    elseif isstruct(x) && isfield(x, 'signals') && isfield(x.signals, 'values')
        A = x.signals.values;
    elseif isstruct(x) && isfield(x, 'Data')
        A = x.Data;
    else
        A = x;
    end

    try
        A = squeeze(double(A));
    catch
        A = squeeze(A);
    end
end

function v = sig2vec(x)
    A = sig2mat(x);
    if isempty(A)
        v = [];
    else
        v = A(:);
    end
end

function A = asRows(x, nCols)
    A = sig2mat(x);
    if isempty(A)
        return;
    end

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

function qOdom = reconstructWheelOdom(t, w, q0, r, d)
    t = t(:);
    N = min(numel(t), size(w,1));
    qOdom = nan(N,3);
    qOdom(1,:) = q0(:).';

    dtDefault = median(diff(t), 'omitnan');

    for k = 2:N
        dt = t(k) - t(k-1);
        if ~isfinite(dt) || dt <= 0
            dt = dtDefault;
        end

        wL = w(k-1,1);
        wR = w(k-1,2);

        v = r/2 * (wR + wL);
        omega = r/d * (wR - wL);

        thetaNew = atan2(sin(qOdom(k-1,3) + omega*dt), ...
                         cos(qOdom(k-1,3) + omega*dt));

        xNew = qOdom(k-1,1) + v*cos(thetaNew)*dt;
        yNew = qOdom(k-1,2) + v*sin(thetaNew)*dt;

        qOdom(k,:) = [xNew, yNew, thetaNew];
    end
end

function [MN,MV,MU,MT] = addMetric(MN,MV,MU,MT,name,value,unit,note)
    MN(end+1,1) = string(name);
    MV(end+1,1) = double(value);
    MU(end+1,1) = string(unit);
    MT(end+1,1) = string(note);
end
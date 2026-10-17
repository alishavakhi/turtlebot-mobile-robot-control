clear; clc; close all;

modelName = 'task2_simulation';

fprintf('\n============================================================\n');
fprintf('VERIFY TASK 2 SIMULINK BLOCK PARAMETERS\n');
fprintf('============================================================\n');

load_system(modelName);

rt = sfroot;

%% ---------- Task Manager code ----------
taskManagerPath = [modelName '/Task_Manager'];
tmChart = rt.find('-isa','Stateflow.EMChart','Path',taskManagerPath);

if isempty(tmChart)
    warning('Could not find Task_Manager as Stateflow.EMChart.');
else
    tmCode = tmChart.Script;

    fprintf('\n================ TASK MANAGER CODE ================\n');
    disp(tmCode);

    fprintf('\n================ TASK MANAGER EXTRACTED VALUES ================\n');
    printRegexValue(tmCode, 'q_wp1 or q_goal_task1', 'q_goal_task1\s*=\s*\[([^\]]+)\]');
    printRegexValue(tmCode, 'q_wp2 or q_goal_task2', 'q_goal_task2\s*=\s*\[([^\]]+)\]');
    printRegexValue(tmCode, 'thresh_wp1_pos / epsilon_1', 'thresh_wp1_pos\s*=\s*([0-9.eE+-]+)');
    printRegexValue(tmCode, 'thresh_wp2_pos / epsilon_2', 'thresh_wp2_pos\s*=\s*([0-9.eE+-]+)');
    printRegexValue(tmCode, 'thresh_wp2_ang / epsilon_theta', 'thresh_wp2_ang\s*=\s*([0-9.eE+-]+)');
    printRegexValue(tmCode, 'goal tolerance', 'dist_threshold\s*=\s*([0-9.eE+-]+)');
    printRegexValue(tmCode, 'WP2 timeout', 'timeout\s*=\s*([0-9.eE+-]+)');
end

%% ---------- Localizer EKF code ----------
ekfPath = [modelName '/Localizer_EKF'];
ekfChart = rt.find('-isa','Stateflow.EMChart','Path',ekfPath);

if isempty(ekfChart)
    warning('Could not find Localizer_EKF as Stateflow.EMChart.');
else
    ekfCode = ekfChart.Script;

    fprintf('\n================ LOCALIZER EKF CODE ================\n');
    disp(ekfCode);

    fprintf('\n================ EKF EXTRACTED VALUES ================\n');
    printRegexValue(ekfCode, 'P0', 'P\s*=\s*eye\(3\)\s*\*\s*([0-9.eE+-]+)');
    printRegexValue(ekfCode, 'Q', 'Q\s*=\s*diag\(\[([^\]]+)\]\)');
    printRegexValue(ekfCode, 'R red', 'R\s*=\s*diag\(\[0\.7,\s*0\.7,\s*0\.9\]\)');
    printRegexValue(ekfCode, 'R normal', 'R\s*=\s*diag\(\[1e-5,\s*1e-5,\s*5e-5\]\)');
    printRegexValue(ekfCode, 'missing Vicon logic', 'all\(Y_vicon\s*==\s*0\)');
end

%% ---------- Helper ----------
function printRegexValue(code, label, pattern)
    token = regexp(code, pattern, 'tokens', 'once');
    if isempty(token)
        fprintf('%s: NOT FOUND. Check manually in the printed code above.\n', label);
    else
        if iscell(token)
            fprintf('%s: %s\n', label, token{1});
        else
            fprintf('%s: %s\n', label, token);
        end
    end
end
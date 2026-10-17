clear; clc;

load('exported_realrobot_test2.mat');

qt = D.q_target;
t  = D.t;

targets = [ ...
   -1.60  -1.10   0;        % Task 1 goal
   -0.20  -1.10   0;        % Task 2 waypoint 1
   -0.20  -0.30   pi/2;     % Task 2 waypoint 2
    1.00  -0.30   0;        % Task 2 final goal
    1.45  -0.30   0;        % Task 3 approach point
    2.00  -0.60   0         % Task 3 circle-start point
];

names = { ...
    'Task1_goal', ...
    'Task2_wp1', ...
    'Task2_wp2', ...
    'Task2_goal', ...
    'Task3_wp1', ...
    'Task3_wp2'};

tol = 1e-9;

for k = 1:size(targets,1)
    idx = find( ...
        abs(qt(:,1)-targets(k,1)) < tol & ...
        abs(qt(:,2)-targets(k,2)) < tol & ...
        abs(qt(:,3)-targets(k,3)) < tol );

    if ~isempty(idx)
        fprintf('\n%s\n', names{k});
        fprintf('  first index = %d, time = %.2f s\n', idx(1), t(idx(1)));
        fprintf('  last  index = %d, time = %.2f s\n', idx(end), t(idx(end)));
    else
        fprintf('\n%s not found exactly.\n', names{k});
    end
end
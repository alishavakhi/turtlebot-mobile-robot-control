% Extracted verbatim from task3_simulation.slx (simulink/stateflow/chart_55.xml) by tools/extract_model_code.py
function [Y_vicon_turtlebot, Y_vicon_ground_truth, ...
          vicon_valid, in_red, in_orange, loss_flag] = Fake_Vicon_FinalActivity(q_true, reset_noise)
%#codegen
% Fake Vicon block consistent with the final activity rules.
%
% Red area: Gaussian noise, sigma_e = 0.02.
% Orange area: measurement loss with p_loss = 0.9.
% Missing measurement is returned as [0;0;0].
% Ground truth is always available only for analysis.

persistent seed

if isempty(seed) || reset_noise > 0.5
    seed = 123456789.0;
end

%% =========================
%  Map coordinates
%  =========================
x5  = -0.60;
x6  =  0.20;
x7  =  1.00;
x11 =  2.90;

y1 = -1.50;
y6 =  0.10;
y8 =  1.50;

%% =========================
%  Parameters
%  =========================
sigma_e = 0.02;     % Gaussian noise standard deviation
p_loss  = 0.90;     % loss probability in orange area

%% =========================
%  True pose
%  =========================
x = q_true(1);
y = q_true(2);
theta = wrapToPiLocal(q_true(3));

Y_vicon_ground_truth = [x; y; theta];

%% =========================
%  Region checks
%  =========================
in_red = double((x >= x5) && (x <= x6) && (y >= y1) && (y <= y6));

in_orange = double((x >= x7) && (x <= x11) && (y >= y1) && (y <= y8));

%% =========================
%  Default output: clean measurement
%  =========================
Y_vicon_turtlebot = Y_vicon_ground_truth;
vicon_valid = 1.0;
loss_flag = 0.0;

%% =========================
%  Apply final-activity measurement rules
%  =========================

if in_orange > 0.5
    % In orange area: measurement is lost with probability p_loss.
    [u, seed] = uniformLCG(seed);

    if u < p_loss
        Y_vicon_turtlebot = [0.0; 0.0; 0.0];
        vicon_valid = 0.0;
        loss_flag = 1.0;
    else
        % Available measurement in orange area.
        % Keep it clean unless you explicitly want extra noise.
        Y_vicon_turtlebot = Y_vicon_ground_truth;
        vicon_valid = 1.0;
        loss_flag = 0.0;
    end

elseif in_red > 0.5
    % In red area: Gaussian corrupted measurement.
    [n1, seed] = gaussianLCG(seed);
    [n2, seed] = gaussianLCG(seed);
    [n3, seed] = gaussianLCG(seed);

    Y_vicon_turtlebot = Y_vicon_ground_truth + sigma_e * [n1; n2; n3];
    Y_vicon_turtlebot(3) = wrapToPiLocal(Y_vicon_turtlebot(3));

    vicon_valid = 1.0;
    loss_flag = 0.0;

else
    % Normal area: clean motion capture.
    Y_vicon_turtlebot = Y_vicon_ground_truth;
    vicon_valid = 1.0;
    loss_flag = 0.0;
end

end

%% ==========================================================
%  Local helper functions
%  ==========================================================

function [u, seed] = uniformLCG(seed)
% Deterministic pseudo-random generator.
% This avoids relying on rand/randn inside Simulink code generation.

a = 1664525.0;
c = 1013904223.0;
m = 4294967296.0;   % 2^32

seed = mod(a*seed + c, m);

% Avoid exactly 0 or 1.
u = (seed + 1.0) / (m + 2.0);
end

function [z, seed] = gaussianLCG(seed)
% Gaussian random number using Box-Muller transform.

[u1, seed] = uniformLCG(seed);
[u2, seed] = uniformLCG(seed);

if u1 < 1e-12
    u1 = 1e-12;
end

z = sqrt(-2.0*log(u1)) * cos(2.0*pi*u2);
end

function a = wrapToPiLocal(a)
a = atan2(sin(a), cos(a));
end

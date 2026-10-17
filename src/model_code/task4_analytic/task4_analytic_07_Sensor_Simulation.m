% Extracted verbatim from task4_simulation_analytic_jacobian.slx (simulink/stateflow/chart_95.xml) by tools/extract_model_code.py
function [v_odo, omega_odo, z_lidar] = Sensor_Simulation(q_true, omega_wheels, reset_noise)
%#codegen
% Simulated encoders, IMU yaw-rate, and LiDAR range measurements

persistent seed

if isempty(seed) || reset_noise > 0.5
    seed = 123456789.0;
end

%% Robot parameters
Ts = 0.04;
r = 0.03;
d = 0.165;

%% Artificial simulation perturbations
sigma_encoder = 0.0008;   % [rad]
sigma_imu_w   = 0.010;    % [rad/s]
sigma_lidar   = 0.006;    % [m]
p_dropout     = 0.03;

%% Wheel encoder / IMU simulation
omega_L = omega_wheels(1);
omega_R = omega_wheels(2);

[n1, seed] = gaussianLCG(seed);
[n2, seed] = gaussianLCG(seed);

dphiL = omega_L*Ts + sigma_encoder*n1;
dphiR = omega_R*Ts + sigma_encoder*n2;

v_enc = r*(dphiL + dphiR)/(2*Ts);
omega_enc = r*(dphiR - dphiL)/(d*Ts);

omega_true = r*(omega_R - omega_L)/d;

[n3, seed] = gaussianLCG(seed);
omega_imu = omega_true + sigma_imu_w*n3;

v_odo = v_enc;
omega_odo = 0.45*omega_enc + 0.55*omega_imu;

%% LiDAR simulation
z_clean = lidarModel(q_true);

z_lidar = zeros(36,1);

for i = 1:36

    if isfinite(z_clean(i))

        [n, seed] = gaussianLCG(seed);
        [u, seed] = uniformLCG(seed);

        if u < p_dropout
            z_lidar(i) = NaN;
        else
            z_lidar(i) = z_clean(i) + sigma_lidar*n;

            if z_lidar(i) < 0.02
                z_lidar(i) = 0.02;
            end
        end

    else

        z_lidar(i) = NaN;

    end

end

end

%% ============================================================
% LiDAR model
% ============================================================
function h = lidarModel(q)

lidar_offset = 0.076;

x = q(1);
y = q(2);
theta = q(3);

pL = [x; y] + lidar_offset*[cos(theta); sin(theta)];

h = NaN(36,1);

for i = 1:36

    angle_rel = -pi + (i-1)*(2*pi/36);
    alpha = theta + angle_rel;

    h(i) = rayBoxDistance(pL, alpha);

end

end

function dist = rayBoxDistance(p, alpha)

cube_xmin = 1.80;
cube_xmax = 2.20;
cube_ymin = -0.20;
cube_ymax = 0.20;

lidar_max_range = 3.0;

dir = [cos(alpha); sin(alpha)];

best = 1e9;
found = false;

%% Vertical walls
if abs(dir(1)) > 1e-10

    tx1 = (cube_xmin - p(1))/dir(1);
    yx1 = p(2) + tx1*dir(2);

    if tx1 > 1e-6 && yx1 >= cube_ymin && yx1 <= cube_ymax
        best = min(best, tx1);
        found = true;
    end

    tx2 = (cube_xmax - p(1))/dir(1);
    yx2 = p(2) + tx2*dir(2);

    if tx2 > 1e-6 && yx2 >= cube_ymin && yx2 <= cube_ymax
        best = min(best, tx2);
        found = true;
    end

end

%% Horizontal walls
if abs(dir(2)) > 1e-10

    ty1 = (cube_ymin - p(2))/dir(2);
    xy1 = p(1) + ty1*dir(1);

    if ty1 > 1e-6 && xy1 >= cube_xmin && xy1 <= cube_xmax
        best = min(best, ty1);
        found = true;
    end

    ty2 = (cube_ymax - p(2))/dir(2);
    xy2 = p(1) + ty2*dir(1);

    if ty2 > 1e-6 && xy2 >= cube_xmin && xy2 <= cube_xmax
        best = min(best, ty2);
        found = true;
    end

end

if found && best <= lidar_max_range
    dist = best;
else
    dist = NaN;
end

end

%% ============================================================
% Random generators
% ============================================================
function [u, seed] = uniformLCG(seed)

a = 1664525.0;
c = 1013904223.0;
m = 4294967296.0;

seed = mod(a*seed + c, m);
u = (seed + 1.0)/(m + 2.0);

end

function [z, seed] = gaussianLCG(seed)

[u1, seed] = uniformLCG(seed);
[u2, seed] = uniformLCG(seed);

if u1 < 1e-12
    u1 = 1e-12;
end

z = sqrt(-2.0*log(u1))*cos(2.0*pi*u2);

end

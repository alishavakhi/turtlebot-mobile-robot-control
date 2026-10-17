% Extracted verbatim from task4_simulation.slx (simulink/stateflow/chart_85.xml) by tools/extract_model_code.py
function [q_target, v_ref, omega_ref, ctrl_mode] = Task_Manager(t)
%#codegen
% Reference Trajectory Manager for Task 4
% Phase 1: Hermite approach trajectory
% Phase 2: circular trajectory around the cube
% Phase 3: stop / hold final pose

%% ============================================================
% Geometry and timing
% ============================================================

% Initial true pose, close to the final configuration of Task 2
q0 = [0.96; -0.34; 4*pi/180];

% Circle start point
x_start = 1.45;
y_start = -0.30;

% Cube / circle center
xc = 2.00;
yc = 0.00;

dx0 = x_start - xc;
dy0 = y_start - yc;

R_circle = sqrt(dx0^2 + dy0^2);

circle_dir = 1.0;
omega_circle_abs = 0.30;      % [rad/s]

N_turns = 1.5;
T_circle = 2*pi*N_turns/omega_circle_abs;

T_approach = 8.0;             % [s]

alpha0 = atan2(dy0, dx0);
theta_start = wrapToPiLocal(alpha0 + circle_dir*pi/2);

q_start = [x_start; y_start; theta_start];

%% Default outputs

q_target = zeros(3,1);
v_ref = 0.0;
omega_ref = 0.0;
ctrl_mode = 2.0;

%% ============================================================
% Phase 1: Hermite approach trajectory
% ============================================================

if t < T_approach

    tau = t/T_approach;

    if tau < 0
        tau = 0;
    elseif tau > 1
        tau = 1;
    end

    % Smooth time law
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    sdot = (30*tau^2 - 60*tau^3 + 30*tau^4)/T_approach;
    sddot = (60*tau - 180*tau^2 + 120*tau^3)/(T_approach^2);

    % Hermite geometry
    p0 = q0(1:2);
    p1 = q_start(1:2);

    L = norm(p1 - p0);

    if L < 0.45
        L = 0.45;
    end

    m0 = L*[cos(q0(3)); sin(q0(3))];
    m1 = L*[cos(q_start(3)); sin(q_start(3))];

    h00 =  2*s^3 - 3*s^2 + 1;
    h10 =      s^3 - 2*s^2 + s;
    h01 = -2*s^3 + 3*s^2;
    h11 =      s^3 -   s^2;

    dh00 =  6*s^2 - 6*s;
    dh10 =  3*s^2 - 4*s + 1;
    dh01 = -6*s^2 + 6*s;
    dh11 =  3*s^2 - 2*s;

    d2h00 =  12*s - 6;
    d2h10 =   6*s - 4;
    d2h01 = -12*s + 6;
    d2h11 =   6*s - 2;

    p = h00*p0 + h10*m0 + h01*p1 + h11*m1;

    dpds = dh00*p0 + dh10*m0 + dh01*p1 + dh11*m1;
    d2pds2 = d2h00*p0 + d2h10*m0 + d2h01*p1 + d2h11*m1;

    pdot = dpds*sdot;
    pddot = d2pds2*(sdot^2) + dpds*sddot;

    v_ref = sqrt(pdot(1)^2 + pdot(2)^2);

    if v_ref > 1e-5

        theta_ref = atan2(pdot(2), pdot(1));

        omega_ref = (pdot(1)*pddot(2) - pdot(2)*pddot(1))/(v_ref^2);

    else

        if tau < 0.5
            theta_ref = q0(3);
        else
            theta_ref = q_start(3);
        end

        omega_ref = 0.0;

    end

    % Avoid unrealistic angular feedforward during very slow motion
    omega_ref = satLocal(omega_ref, -0.80, 0.80);

    q_target = [p(1); p(2); wrapToPiLocal(theta_ref)];

    ctrl_mode = 2.0;

%% ============================================================
% Phase 2: circular trajectory tracking
% ============================================================

elseif t < T_approach + T_circle

    elapsed_circle = t - T_approach;

    alpha = alpha0 + circle_dir*omega_circle_abs*elapsed_circle;

    xd = xc + R_circle*cos(alpha);
    yd = yc + R_circle*sin(alpha);
    thd = wrapToPiLocal(alpha + circle_dir*pi/2);

    q_target = [xd; yd; thd];

    v_ref = R_circle*omega_circle_abs;
    omega_ref = circle_dir*omega_circle_abs;

    ctrl_mode = 2.0;

%% ============================================================
% Phase 3: stop / hold final pose
% ============================================================

else

    alpha = alpha0 + circle_dir*omega_circle_abs*T_circle;

    xd = xc + R_circle*cos(alpha);
    yd = yc + R_circle*sin(alpha);
    thd = wrapToPiLocal(alpha + circle_dir*pi/2);

    q_target = [xd; yd; thd];

    v_ref = 0.0;
    omega_ref = 0.0;

    ctrl_mode = 3.0;

end

end

function y = satLocal(u, umin, umax)

y = min(max(u, umin), umax);

end

function a = wrapToPiLocal(a)

a = atan2(sin(a), cos(a));

end

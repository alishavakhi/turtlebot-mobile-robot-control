% Extracted verbatim from task4_simulation_analytic_jacobian.slx (simulink/stateflow/chart_74.xml) by tools/extract_model_code.py
function [q_dot, u_actual] = q_dot(q, omega_wheels, r, d)
u_actual = [r/2, r/2;-r/d, r/d]*omega_wheels;
theta = q(3);
q_dot = [[cos(theta), 0];[sin(theta), 0];[0,1]]*u_actual;

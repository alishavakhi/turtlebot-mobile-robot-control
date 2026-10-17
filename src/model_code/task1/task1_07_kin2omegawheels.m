% Extracted verbatim from task1_simulation.slx (simulink/stateflow/chart_71.xml) by tools/extract_model_code.py
function omega_wheels = kin2omegawheels(u, r, d)
omega_l = u(1)/r-d/(2*r)*u(2);
omega_r = u(1)/r+d/(2*r)*u(2);
omega_wheels = [omega_l;omega_r];

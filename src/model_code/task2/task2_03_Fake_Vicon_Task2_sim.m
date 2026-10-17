% Extracted verbatim from task2_simulation.slx (simulink/stateflow/chart_38.xml) by tools/extract_model_code.py
function Y_vicon_turtlebot_sim = Fake_Vicon_Task2_sim(q_actual)
%#codegen

x  = q_actual(1);
y  = q_actual(2);
th = q_actual(3);

Y_vicon_turtlebot_sim = [x; y; th];

x5 = -0.60;
x6 =  0.20;
y1 = -1.50;
y6 =  0.10;

sigma_e = 0.02;

in_red = (x >= x5) && (x <= x6) && (y >= y1) && (y <= y6);

if in_red
    Y_vicon_turtlebot_sim = [x; y; th] + sigma_e * randn(3,1);
end

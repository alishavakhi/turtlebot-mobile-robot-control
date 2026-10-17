% Extracted verbatim from task2_simulation.slx (simulink/stateflow/chart_51.xml) by tools/extract_model_code.py
function [y, condition] = check(pos)

x1 = -2.9; y1 = -1.5;
x2 = -1.6; y2 = -1.1;
x3 = -1.5; y3 = -0.7;
x4 = -1.4; y4 = -0.3;
x5 = -0.6; y5 =  0.1;
x6 =  0.2; y6 =  1.5;
x7 =  1.0;
x8 =  1.1;
x9 =  2.9;

P_loss = 0.9;


if ((pos(1) >= x5) && (pos(1) <= x6)) && ((pos(2) >= y1) && (pos(2) <= y5))
    % add noise
    y = pos + 0.02*randn(3,1);
    condition = 1;
elseif ((pos(1) >= x7) && (pos(1) <= x9)) && ((pos(2) >= y1) && (pos(2) <= y6))
    % measurement loss
    if rand > P_loss
        y = pos;
    else
        y = 0*pos;
    end
    condition = 2;
else
    y = pos;
    condition = 0;
end

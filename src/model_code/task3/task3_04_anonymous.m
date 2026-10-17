% Extracted verbatim from task3_simulation.slx (simulink/stateflow/chart_68.xml) by tools/extract_model_code.py
function visualizer(q_est)
    coder.extrinsic('draw_robot');
    draw_robot(q_est(1), q_est(2));
end

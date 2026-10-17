% Extracted verbatim from online_robot_2024b.slx (simulink/stateflow/chart_163.xml) by tools/extract_model_code.py
function u = fcn(q, q_d, dq_d, u_d, e_world_RF, controller_index, control_par)
% compute the error in the unicycle RF
    e = zeros(3,1);
    e(1) = cos(q(3))*e_world_RF(1) + sin(q(3))*e_world_RF(2);
    e(2) = -sin(q(3))*e_world_RF(1) + cos(q(3))*e_world_RF(2);
    % e(3) = e_world_RF(3);
    e(3) = atan2(sin(e_world_RF(3)),...
             cos(e_world_RF(3)));
    u_ff = zeros(2,1);
    u_k = zeros(2,1);
    u = zeros(2,1);

    %FBL
    pbd_dot = zeros(2,1);
    pbd_dot(1) = dq_d(1) - control_par(3)*sin(q_d(3))*dq_d(3);
    pbd_dot(2) = dq_d(1) + control_par(3)*cos(q_d(3))*dq_d(3);
    xbd = q_d(1)+control_par(3)*cos(q_d(3));
    ybd = q_d(2)+control_par(3)*sin(q_d(3));
    xb = q(1)+control_par(3)*cos(q(3)); 
    yb = q(2)+control_par(3)*sin(q(3));

    e_tilde = zeros(2,1);
    e_tilde(1) = xbd -xb;
    e_tilde(2) = ybd - yb;


    

    
if controller_index == 1
    % linear controller
    k1 = 2*control_par(2)*control_par(1);
    v_safe = max(abs(u_d(1)), 1e-3);
    k2 = (control_par(2)^2 - u_d(2)^2) / v_safe;
    k3 = k1;
      
    u_ff(1)=u_d(1)*cos(e(3));
    u_ff(2)=u_d(2);
   
    u_k(1)=-e(1)*k1;

   if abs(e(3)) < 1e-6
        sinc_e = 1;
   else
        sinc_e = sin(e(3))/e(3);
   end
    
    u_k(2) = -k2*(u_d(1)*sinc_e)*e(2) - k3*e(3);
    u(1)=u_ff(1) - u_k(1);
    u(2)=u_ff(2) - u_k(2);
   
elseif controller_index == 2
    % nonlinear controller
    u_d(1) = max(u_d(1),1e-3);
    a = sqrt(u_d(1)*control_par(2) + u_d(2)^2);
    k1 = 2*a*control_par(1);
    k2 = control_par(2);
    k3 = k1;
    % feedforward
    u_ff(1)=u_d(1)*cos(e(3));
    u_ff(2)=u_d(2);

    u_k(1)=-k1*e(1);
    if abs(e(3)) < 1e-6
        sinc_e = 1;
    else
        sinc_e = sin(e(3))/e(3);
    end
    
    u_k(2) = -k2*(u_d(1)*sinc_e)*e(2) - k3*e(3);

    u(1)=u_ff(1) - u_k(1);
    u(2)=u_ff(2) - u_k(2);
    
elseif controller_index == 3
    % feedback linearization
    k1 = control_par(1);
    k2 = control_par(2);
    b = control_par(3);
    K = diag([k1,k2]);
    % u_k = [cos(q(3)), sin(q(3));...
    %     -sin(q(3))/b, cos(q(3))/b]*[pbd_dot(1)+k1*(xbd -xb); pbd_dot(2)+k2*(ybd -yb)];
    T = [cos(q(3)), -b*sin(q(3)); sin(q(3)), b*cos(q(3))];
    temp = (pbd_dot+K*e_tilde);

    u = T\temp;
    
else
    u = u_d;
end
if norm(q_d(1:2) - q(1:2)) < 0.001 && norm(u_d) < 1e-6
    u = [0; 0];
end

%% Saturation
v_max = 0.35;
w_max = 2.0;
u(1) = max(min(u(1), v_max), -v_max);
u(2) = max(min(u(2), w_max), -w_max);
end

% Extracted verbatim from task1_simulation.slx (simulink/stateflow/chart_52.xml) by tools/extract_model_code.py
function q_est = fcn(v, omega, Y_vicon, Ts, q_init, is_real_robot)
    % Extended Kalman Filter with Mode-Aware Initialization
    persistent q_hat P Q initialized
    
    % --- ۱. مقداردهی اولیه هوشمند ---
    if isempty(initialized)
        if is_real_robot == 1
            % در حالت واقعی: تا Vicon دیتا نداده، صبر کن
            if all(Y_vicon == 0)
                q_est = q_init; % یا خروجی فایلت را بده تا دیتا برسد
                return;
            else
                q_hat = Y_vicon; % اولین دیتای معتبر Vicon را قفل کن
                initialized = true;
            end
        else
            % در حالت شبیه‌سازی: بلافاصله با q_init استارت بزن
            q_hat = q_init;
            initialized = true;
        end
        P = eye(3) * 0.1; 
        Q = diag([1e-4, 1e-4, 5e-4]); % اعتماد به ادومتری
    end

    % --- ۲. پیش‌بینی (Prediction) ---
    theta = q_hat(3);
    % مدل کینماتیک غیرخطی یونیسایکل
    q_pred = q_hat + [v*cos(theta); v*sin(theta); omega] * Ts;
    q_pred(3) = wrapToPi(q_pred(3));

    % محاسبه ژاکوبیان (خطی‌سازی)
    F = [1, 0, -v*sin(theta)*Ts;
         0, 1,  v*cos(theta)*Ts;
         0, 0,  1];
    P = F * P * F' + Q;

    % --- ۳. اصلاح (Update) ---
    % تشخیص ناحیه قرمز (نویز سنسور)
    in_red = (q_pred(1) >= -0.6 && q_pred(1) <= 0.2);
    is_lost = all(Y_vicon == 0);

    if in_red
        R = diag([0.7, 0.7, 0.9]); % واریانس بالا برای ناحیه نویزی
    else
        R = diag([1e-5, 1e-5, 5e-5]); % واریانس پایین برای ناحیه امن
    end

    if is_lost
        % تسک ۳: سنسور قطع است، فقط ادومتری
        q_est = q_pred;
    else
        % فیلتر کالمن استاندارد
        H = eye(3);
        K = (P * H') / (H * P * H' + R);
        
        innovation = Y_vicon - q_pred;
        innovation(3) = wrapToPi(innovation(3)); % اصلاح زاویه در تفاضل
        
        q_est = q_pred + K * innovation;
        q_est(3) = wrapToPi(q_est(3));
        P = (eye(3) - K * H) * P;
    end
    
    q_hat = q_est;
end

% Extracted verbatim from task3_simulation.slx (simulink/stateflow/chart_94.xml) by tools/extract_model_code.py
function y = manual_unwrap(u)
    % این تابع به صورت دستی عملیات Unwrap را برای سیگنال‌های ورودی انجام می‌دهد.
    
    persistent prev_u cumulative_offset
    
    % مقداردهی اولیه در اولین گام اجرای شبیه‌سازی
    if isnan(u)
        u = 0;
    end
    if isempty(prev_u)
        prev_u = u;
        cumulative_offset = zeros(size(u));
    end
    
    % محاسبه اختلاف زاویه فعلی با قبلی
    diff = u - prev_u;
    
    % شناسایی پرش‌های بزرگتر از 180 درجه (pi رادیان)
    for i = 1:length(u)
        if diff(i) > pi
            cumulative_offset(i) = cumulative_offset(i) - 2*pi;
        elseif diff(i) < -pi
            cumulative_offset(i) = cumulative_offset(i) + 2*pi;
        end
    end
    
    % اعمال آفست تجمعی به ورودی فعلی
    y = u + cumulative_offset;
    
    % ذخیره مقدار فعلی برای استفاده در گام بعدی
    prev_u = u;

end

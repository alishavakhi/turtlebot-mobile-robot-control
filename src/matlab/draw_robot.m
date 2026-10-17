function draw_robot(x, y)
    coder.extrinsic('figure', 'plot', 'hold', 'axis', 'patch', 'drawnow', 'isvalid', 'set', 'gca', 'gcf');
    persistent fig robot_marker path_line x_hist y_hist
    
    if isempty(fig) || ~isvalid(fig)
        fig = figure('Name', 'TurtleBot Final Activity Map', 'NumberTitle', 'off');
        
        % تنظیم رنگ حاشیه و رنگ داخل نمودار به سفید خالص
        set(gcf, 'Color', 'w'); 
        set(gca, 'Color', 'w');
        
        hold on; grid on; 
        
        % کم‌رنگ کردن خطوط گرید که چشم رو اذیت نکنه
        set(gca, 'GridColor', 'k', 'GridAlpha', 0.1); 
        axis equal;
        axis([-3.5 3.5 -2.0 2.0]);
        
        % ۱. ناحیه شروع (INITIAL POSITION - آبی روشن)
        patch([-2.90, -1.50, -1.50, -2.90], [0.10, 0.10, 1.50, 1.50], [0.75, 0.85, 0.95], 'EdgeColor', 'b', 'LineStyle', '--');
        
        % ۲. ناحیه قرمز
        patch([-0.60, 0.20, 0.20, -0.60], [-1.50, -1.50, 0.10, 0.10], [1, 0.7, 0.7], 'EdgeColor', 'none');
        
        % ۳. ناحیه نارنجی
        patch([1.00, 2.90, 2.90, 1.00], [-1.50, -1.50, 1.50, 1.50], [0.95, 0.6, 0.3], 'EdgeColor', 'none');
        
        % ۴. دیوارهای بیرونی (کادر مشکی ضخیم)
        plot([-2.90, 2.90, 2.90, -2.90, -2.90], [-1.50, -1.50, 1.50, 1.50, -1.50], 'k-', 'LineWidth', 4);
        
        % ۵. مانع ۱ (باکس طوسی سمت چپ)
        patch([-1.50, -0.60, -0.60, -1.50], [-0.70, -0.70, 0.10, 0.10], [0.65, 0.65, 0.65], 'EdgeColor', 'k', 'LineWidth', 3);
        
        % ۶. مانع ۲ (باکس طوسی پایین)
        patch([0.20, 1.00, 1.00, 0.20], [-1.50, -1.50, -0.70, -0.70], [0.65, 0.65, 0.65], 'EdgeColor', 'k', 'LineWidth', 3);
        
        % ۷. مکعب وسط ناحیه نارنجی (قهوه‌ای تیره)
        patch([1.80, 2.20, 2.20, 1.80], [-0.20, -0.20, 0.20, 0.20], [0.5, 0.4, 0.3], 'EdgeColor', 'k', 'LineWidth', 3);
        
        % ۸. نقاط هدف
        plot(-1.60, -1.10,'kx', 'MarkerSize', 12, 'LineWidth', 3); % هدف 1 (مثلث)
        plot(1.00, -0.30, 'kx', 'MarkerSize', 12, 'LineWidth', 3); % هدف 2 (ضربدر)
        plot(1.45, -0.30, 'kx', 'MarkerSize', 12, 'LineWidth', 3);
        % مقداردهی اولیه ربات
        robot_marker = plot(x, y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        path_line = plot(x, y, 'b-', 'LineWidth', 1.5);
        
        x_hist = x; y_hist = y;
    else
        x_hist = [x_hist, x];
        y_hist = [y_hist, y];
        set(robot_marker, 'XData', x, 'YData', y);
        set(path_line, 'XData', x_hist, 'YData', y_hist);
    end
    drawnow limitrate;
end
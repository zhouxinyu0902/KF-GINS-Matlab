function gps = visualize_gpgga_data(rw)
    % 可视化GPGGA数据
    rw=rw';
    gnss_seconds = rw(:, 19);       % UTC周秒 (s)
    time = gnss_seconds(:)-gnss_seconds(1);

    latitude = rw(:, 4) + rw(:, 5)/60;   
    longitude = rw(:, 7) + rw(:, 8)/60;  
    gps_status = rw(:, 10);
    satellites_used = rw(:, 11);
    hdop = rw(:, 12);
    altitude = rw(:, 13) + rw(:, 15);
    
    gps = [gnss_seconds,time,latitude,longitude,altitude,gps_status];
    % 创建图形窗口
    figure('Name', 'GPGGA数据可视化分析', 'Position', [100, 50, 1400, 900]);
    % 1. 地理轨迹显示
    subplot(2, 3, 1);
    lat=deg2km(latitude)-deg2km(latitude(1));
    lon=deg2km(longitude)-deg2km(longitude(1));
    plot(lon,lat,'DisplayName','轨迹');
    hold on
    plot(lon(1),lat(1),'.','DisplayName','起点')
    hold on
    plot(lon(end),lat(end),'.','DisplayName','终点')
    axis equal
    xlabel('East (km)');
    ylabel('North (km)');
    legend();
    grid on;
    
    % 2. 3D运动轨迹
    subplot(2, 3, 2);
    plot3(longitude, latitude, altitude, 'b-', 'LineWidth', 1.5,'DisplayName','轨迹');
    hold on;
    scatter3(longitude(1), latitude(1), altitude(1), 100, 'g', 'filled', 'DisplayName', '起点');
    scatter3(longitude(end), latitude(end), altitude(end), 100, 'r', 'filled', 'DisplayName', '终点');
    title('3D运动轨迹');
    xlabel('经度 (°)');
    ylabel('纬度 (°)');
    zlabel('高度 (m)');
    legend('Location', 'best');
    grid on;
    view(-30, 30);
    
    % 3. 卫星数量和时间关系
    subplot(2, 3, 3);
    plot(time, satellites_used, 'bo-', 'MarkerSize', 3);
    xlabel('时间 (小时)');
    ylabel('使用卫星数量');
    title('卫星数量随时间变化');
    grid on;
    
    % 4. 高度剖面
    subplot(2, 3, 4);
    plot(time, altitude, 'r-', 'LineWidth', 1.5);
    xlabel('时间 (小时)');
    ylabel('海拔高度 (m)');
    title('海拔高度剖面');
    grid on;
    
    % 5. HDOP值分析
    subplot(2, 3, 5);
    plot(time, hdop, 'm-', 'LineWidth', 1.5);
    xlabel('时间 (小时)');
    ylabel('HDOP值');
    title('水平精度因子(HDOP)');
    grid on;
    
    % 6. 数据质量统计
    subplot(2, 3, 6);
    axis off;
    
    % 计算统计信息
    valid_positions = sum(~isnan(latitude));
    avg_satellites = mean(satellites_used);
    avg_hdop = mean(hdop);
    max_altitude = max(altitude);
    min_altitude = min(altitude);
    
    text(0.1, 0.9, '===== GPGGA数据质量统计 =====', 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.75, sprintf('总记录数: %d', length(time)));
    text(0.1, 0.60, sprintf('有效定位数: %d (%.1f%%)', valid_positions, valid_positions/length(time)*100));
    text(0.1, 0.45, sprintf('平均卫星数: %.1f', avg_satellites));
    text(0.1, 0.30, sprintf('平均HDOP: %.2f', avg_hdop));
    text(0.1, 0.15, sprintf('高度范围: %.1f - %.1f m', min_altitude, max_altitude));
    
    % % 保存图形
    % saveas(gcf, 'gpgga_data_analysis.png');
end
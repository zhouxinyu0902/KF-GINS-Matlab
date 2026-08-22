function [imu,pva,pv_gnss]=visualize_auax_navigation_results(rw, time)
    % AUAX协议导航结果可视化函数
    % 输入参数:
    %   rw - 导航数据矩阵，按AUAX协议表格的编号顺序排列
    %   time - 时间向量
       
    % 根据AUAX协议表格提取数据
    run_time = rw(:, 1);          % 运行时间 (s)
    rw(:, 2) = rw(:, 2);       % GPS周秒 (s)，在外部已经加上了18s
    heading = rw(:, 3);           % 航向角 (°)
    pitch = rw(:, 4);             % 俯仰角 (°)
    roll = rw(:, 5);              % 横滚角 (°)
    latitude = rw(:, 6);          % 纬度 (°)
    longitude = rw(:, 7);         % 经度 (°)
    altitude = rw(:, 8);          % 高度 (m)
    vel_n = rw(:, 9);             % 北向速度 (m/s)
    vel_e = rw(:, 10);            % 东向速度 (m/s)
    est_error = rw(:, 11);        % 估计误差 (°)
    gyro_x = rw(:, 12);           % X角速率 (°/s)
    gyro_y = rw(:, 13);           % Y角速率 (°/s)
    gyro_z = rw(:, 14);           % Z角速率 (°/s)
    accel_x = rw(:, 15);          % X轴比力 (g)
    accel_y = rw(:, 16);          % Y轴比力 (g)
    accel_z = rw(:, 17);          % Z轴比力 (g)
    temperature = rw(:, 18);      % 系统温度 (℃)
    ref_lat = rw(:, 19);          % 参考纬度 (°)
    ref_lon = rw(:, 20);          % 参考经度 (°)
    ref_alt = rw(:, 21);          % 参考高度 (m)
    ref_vel_n = rw(:, 22);        % 参考北速 (m/s)
    ref_vel_e = rw(:, 23);        % 参考东速 (m/s)
    gnss_heading = rw(:, 24);     % GNSS航向 (°)
    gnss_satellites = rw(:, 25);  % GNSS星数信息
    status_word = uint32(rw(:, 32));      % 流程控制状态字
    
    % 提取数据
    imu = rw(:,[12:17,1:2]);
    heading_new = mod(-heading, 360);
    pva=[rw(:,[1:2,6:10]),zeros(size(rw(:, 6))),roll,pitch,heading_new];
    pv_gnss=[rw(:,[1:2,19:23]),zeros(size(rw(:, 6)))];
    % 计算衍生数据
    vel_horizontal = sqrt(vel_n.^2 + vel_e.^2);
    vel_total = sqrt(vel_n.^2 + vel_e.^2);
    
    % 提取GNSS信息
    % gnss_mode = bitand(bitshift(gnss_satellites, -8), 0xFF);  % GGA定位模式
    % gnss_usable_sats = bitand(gnss_satellites, 0xFF);        % 可用星数
    % 创建图形窗口
    myfigurestartup(10,5,'prese')
    % 1. 地理轨迹显示
    subplot(2, 2, 1);
    lat=deg2km(latitude)-deg2km(latitude(1));
    lon=deg2km(longitude)-deg2km(longitude(1));
    %%%%%%%%%%%%%% 计算距离长度 %%%%%%%%%%%%%%
    dlat=diff(lat);
    dlon=diff(lon);
    distance_sum=sum(sqrt(dlat.^2+dlon.^2));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    subplot(2, 2, 2);
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
    
    % 3. 姿态角分析
    subplot(2, 2, 3);
    plot(time, heading, 'r-', 'LineWidth', 1.5, 'DisplayName', '航向角');
    hold on;
    plot(time, pitch, 'g-', 'LineWidth', 1.5, 'DisplayName', '俯仰角');
    plot(time, roll, 'b-', 'LineWidth', 1.5, 'DisplayName', '横滚角');
    title('姿态角变化');
    xlabel('时间 (s)');
    ylabel('角度 (°)');
    legend('Location', 'best');
    grid on;
    
    % 4. 速度分析
    subplot(2, 2, 4);
    plot(time, vel_n, 'r-', 'LineWidth', 1.5, 'DisplayName', '北向速度');
    hold on;
    plot(time, vel_e, 'g-', 'LineWidth', 1.5, 'DisplayName', '东向速度');
    plot(time, vel_horizontal, 'b-', 'LineWidth', 2, 'DisplayName', '水平速度');
    title('速度分量分析');
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    legend('Location', 'best');
    grid on;
    
    % % 5. IMU传感器数据
    % figure
    % subplot(2, 2, 1);
    % yyaxis left;
    % plot(time, gyro_x, 'r-', 'DisplayName', 'X角速率');
    % hold on;
    % plot(time, gyro_y, 'g-', 'DisplayName', 'Y角速率');
    % plot(time, gyro_z, 'b-', 'DisplayName', 'Z角速率');
    % ylabel('角速率 (°/s)');
    % 
    % yyaxis right;
    % plot(time, accel_x, 'r--', 'DisplayName', 'X比力');
    % plot(time, accel_y, 'g--', 'DisplayName', 'Y比力');
    % plot(time, accel_z, 'b--', 'DisplayName', 'Z比力');
    % ylabel('比力 (g)');
    % title('IMU传感器数据');
    % xlabel('时间 (s)');
    % grid on;
    % 
    % % 6. 高度和温度剖面
    % subplot(2, 2, 2);
    % yyaxis left;
    % plot(time, altitude, 'b-', 'LineWidth', 1.5, 'DisplayName', '高度');
    % ylabel('高度 (m)');
    % 
    % yyaxis right;
    % plot(time, temperature, 'r-', 'LineWidth', 1.5, 'DisplayName', '温度');
    % ylabel('温度 (℃)');
    % title('高度和温度剖面');
    % xlabel('时间 (s)');
    % grid on;
    % 
    % % 7. GNSS信息分析
    % subplot(2, 2, 3);
    % yyaxis left;
    % plot(time, gnss_heading, 'b-', 'LineWidth', 1.5, 'DisplayName', 'GNSS航向');
    % ylabel('GNSS航向 (°)');
    % 
    % % yyaxis right;
    % % plot(time, gnss_usable_sats, 'ro-', 'LineWidth', 1.5, 'MarkerSize', 3, 'DisplayName', '可用星数');
    % % ylabel('卫星数量');
    % % title('GNSS信息分析');
    % xlabel('时间 (s)');
    % grid on;
    % 
    % % 8. 流程控制状态字
    % subplot(2, 2, 4);
    % plot(time, status_word, 'bo-', 'MarkerSize', 3, 'LineWidth', 1);
    % title('流程控制状态字');
    % xlabel('时间 (s)');
    % ylabel('状态字值');
    % grid on;
    
    % 9. 参考数据对比
    figure
    subplot(2, 2, 1);
    plot(time, vel_n, 'r-', 'LineWidth', 1.5, 'DisplayName', '北向速度');
    hold on;
    plot(time, ref_vel_n, 'r--', 'LineWidth', 1.5, 'DisplayName', '参考北速');
    plot(time, vel_e, 'g-', 'LineWidth', 1.5, 'DisplayName', '东向速度');
    plot(time, ref_vel_e, 'g--', 'LineWidth', 1.5, 'DisplayName', '参考东速');
    title('速度与参考对比');
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    legend('Location', 'best');
    grid on;
    
    % 10. 位置精度分析
    subplot(2, 2, 2);
    lat_error = abs(latitude - ref_lat) * 111319.9;
    lon_error = abs(longitude - ref_lon) * 111319.9 .* cosd(latitude);
    alt_error = abs(altitude - ref_alt);
    
    plot(time, lat_error, 'r-', 'DisplayName', '纬度误差');
    hold on;
    plot(time, lon_error, 'g-', 'DisplayName', '经度误差');
    plot(time, alt_error, 'b-', 'DisplayName', '高度误差');
    title('位置精度分析');
    xlabel('时间 (s)');
    ylabel('误差 (m)');
    legend('Location', 'best');
    grid on;
    
    % 11. 估计误差分析
    subplot(2, 2, 3);
    plot(time, est_error, 'm-', 'LineWidth', 1.5);
    title('估计误差变化');
    xlabel('时间 (s)');
    ylabel('估计误差 (°)');
    grid on;
    
    % 12. 统计信息显示
    subplot(2, 2, 4);
    axis off;
    
    % 计算性能统计
    % total_distance = calculate_auax_distance(longitude, latitude);
    total_time = time(end) - time(1);
    avg_speed = mean(vel_horizontal);
    max_speed = max(vel_horizontal);
    % avg_gnss_sats = mean(gnss_usable_sats);
    
    text(0.1, 0.9, '===== AUAX协议导航性能统计 =====', 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.75, sprintf('总行程距离: %.2f km', distance_sum));
    text(0.1, 0.60, sprintf('总时长: %.1f s', total_time));
    text(0.1, 0.45, sprintf('平均速度: %.2f m/s', avg_speed));
    text(0.1, 0.30, sprintf('最大速度: %.2f m/s', max_speed));
    text(0.1, 0.15, sprintf('高度范围: %.1f - %.1f m', min(altitude), max(altitude)));
    % text(0.1, 0.3, sprintf('GNSS平均星数: %.1f', avg_gnss_sats));
    % text(0.1, 0.2, sprintf('平均估计误差: %.2f°', mean(est_error)));
    text(0.1, 0, sprintf('系统平均温度: %.1f℃', mean(temperature)));
    
    % 保存图形
    % saveas(fig, 'auax_navigation_analysis.png');
end



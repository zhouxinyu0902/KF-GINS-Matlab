function [imu,pva,posstd]=visualize_gpchcx_navigation_results(rw)
    % gpchcx协议导航结果可视化函数
    % 输入参数:
    %   rw - 导航数据矩阵，按协议表格的编号顺序排列[46×N]
    %   time - 时间向量
    rw=rw';
    % 根据gpchcx协议表格提取数据
    week = rw(:, 1);
    gnss_seconds = rw(:, 2);       % UTC周秒 (s)
    time = gnss_seconds(:)-gnss_seconds(1);
    heading = rw(:, 3);           % 航向角 (°)
    pitch = rw(:, 4);             % 俯仰角 (°)
    roll = rw(:, 5);              % 横滚角 (°)
    gyro_x = rw(:, 6);           % X角速率 (°/s), 车辆坐标系下、含算法零偏补偿
    gyro_y = rw(:, 7);           % Y角速率 (°/s)
    gyro_z = rw(:, 8);           % Z角速率 (°/s)
    accel_x = rw(:, 9);          % X轴比力 (g), 不含重力补偿，车辆坐标系下、含算法零偏补偿
    accel_y = rw(:, 10);          % Y轴比力 (g)
    accel_z = rw(:, 11);          % Z轴比力 (g)
    latitude = rw(:, 12);          % 纬度 (°)
    longitude = rw(:, 13);         % 经度 (°)
    altitude = rw(:, 14);          % 高度 (m)
    vel_e = rw(:, 15);             % 东向速度 (m/s)
    vel_n = rw(:, 16);            % 北向速度 (m/s)
    vel_u = rw(:, 17);            % 天向速度 (m/s)
    V_2D = rw(:, 18);             % 平面速度 (m/s)

    
    NSV1 = rw(:, 19);          % 主天线1卫星数
    NSV2 = rw(:, 20);          % 副天线2卫星数
    NSV1_used = rw(:, 43);          % 主天线1使用卫星数
    NSV2_used = rw(:, 44);          % 副天线2使用卫星数

    Status = rw(:, 21);          % 状态字
    age = rw(:, 22);             % 差分延时
    warning = rw(:, 23);        

    Lattitude_std=rw(:, 24);  % 纬度标准差
    Longitude_std=rw(:, 25);  % 经度标准差
    Altitude_std=rw(:, 26);  % 高度标准差

    Ve_std = rw(:, 27);
    Vn_std = rw(:, 28);
    Vu_std = rw(:, 29);
    Roll_std = rw(:, 30);
    Pitch_std = rw(:, 31);
    Heading_std = rw(:, 32);

    % 提取数据
    imu=rw(:,[6:11,2]);
    
    % 角度转换
    % heading_new = 360 - heading;
    % heading_new(heading_new > 180) = heading_new(heading_new > 180) - 360;
    pva = [rw(:,[1:2,12:14]),vel_n,vel_e,-vel_u, roll, pitch, heading];
    posstd = [gnss_seconds,Lattitude_std,Longitude_std,Altitude_std,Vn_std,Ve_std,Vu_std];
    % 计算衍生数据
    vel_horizontal = sqrt(vel_n.^2 + vel_e.^2);
    vel_std_total_2d = sqrt(Ve_std.^2 + Vn_std.^2);
    position_std_2d = sqrt(Lattitude_std.^2 + Longitude_std.^2);
    % 创建图形窗口
    myfigurestartup(10,5,'prese');
    % 1. 地理轨迹显示
    subplot(2, 2, 1);
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
    legend('Location','best');
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
    hold on;
    plot(time, vel_horizontal, 'b-', 'LineWidth', 2, 'DisplayName', '水平速度计算');
    hold on;
    plot(time, V_2D, 'b', 'LineWidth', 2, 'DisplayName', '水平速度参考');
    title('速度分量分析');
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    legend('Location', 'best');
    grid on;
    
    % % 5. IMU传感器数据
    % figure
    % subplot(2, 1, 1);
    % 
    % plot(time, gyro_x, 'r-', 'DisplayName', 'X角速率');
    % hold on;
    % plot(time, gyro_y, 'g-', 'DisplayName', 'Y角速率');
    % plot(time, gyro_z, 'b-', 'DisplayName', 'Z角速率');
    % ylabel('角速率 (°/s)');
    % xlabel('时间 (s)');
    % grid on;
    % 
    % subplot(2, 1, 2);
    % plot(time, accel_x, 'r--', 'DisplayName', 'X比力');
    % plot(time, accel_y, 'g--', 'DisplayName', 'Y比力');
    % plot(time, accel_z, 'b--', 'DisplayName', 'Z比力');
    % ylabel('比力 (g)');
    % title('IMU传感器数据');
    % xlabel('时间 (s)');
    % grid on;

    
    % 6. 误差绘图
    myfigurestartup(10,5,'prese');
    subplot(2, 2, 1);
    plot(time, Ve_std, 'r', 'LineWidth', 1.5, 'DisplayName', '东向速度误差');
    hold on;
    plot(time, Vn_std, 'g', 'LineWidth', 1.5, 'DisplayName', '北向速度误差');
    plot(time, Vu_std, 'b', 'LineWidth', 1.5, 'DisplayName', '天向速度误差');
    plot(time, vel_std_total_2d, 'r--', 'LineWidth', 1.5, 'DisplayName', '水平速度误差');
    title('速度与参考对比');
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    legend('Location', 'best');
    grid on;
    
    % 7. 位置精度分析
    subplot(2, 2, 2);   
    plot(time, Lattitude_std, 'r-', 'DisplayName', '纬度误差');
    hold on;
    plot(time, Longitude_std, 'g-', 'DisplayName', '经度误差');
    plot(time, Altitude_std, 'b-', 'DisplayName', '高度误差');
    plot(time, position_std_2d, 'b--', 'DisplayName', '水平位置误差');
    title('位置精度分析');
    xlabel('时间 (s)');
    ylabel('误差 (m)');
    legend('Location', 'best');
    grid on;
    
    % 11. 姿态误差分析
    subplot(2, 2, 3);   
    plot(time, Heading_std, 'r-', 'DisplayName', '航向角误差');
    hold on;
    plot(time, Roll_std, 'g-', 'DisplayName', '横滚角误差');
    plot(time, Pitch_std, 'b-', 'DisplayName', '俯仰角误差');

    title('姿态精度分析');
    xlabel('时间 (s)');
    ylabel('误差 (m)');
    legend('Location', 'best');
    grid on;
    
    % 12. 统计信息显示
    subplot(2, 2, 4);
    axis off;
    
    % 计算性能统计
    total_distance = calculate_auax_distance(longitude, latitude);
    %%%%%%%%%%%%%% 计算距离长度 %%%%%%%%%%%%%%
    % dlat = diff(lat);
    % dlon = diff(lon);
    % total_distance=sum(sqrt(dlat.^2+dlon.^2));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    total_time = time(end) - time(1);
    avg_speed = mean(vel_horizontal);
    max_speed = max(vel_horizontal);
    % avg_gnss_sats = mean(gnss_usable_sats);
    
    text(0.1, 0.9, '===== GPCHCX协议导航性能统计 =====', 'FontSize', 12, 'FontWeight', 'bold');
    text(0.1, 0.75, sprintf('总行程距离: %.2f km', total_distance));
    text(0.1, 0.60, sprintf('总时长: %.1f s', total_time));
    text(0.1, 0.45, sprintf('平均速度: %.2f m/s', avg_speed));
    text(0.1, 0.30, sprintf('最大速度: %.2f m/s', max_speed));
    text(0.1, 0.15, sprintf('高度范围: %.1f - %.1f m', min(altitude), max(altitude)));
    % text(0.1, 0.3, sprintf('GNSS平均星数: %.1f', avg_gnss_sats));
    % text(0.1, 0.2, sprintf('平均估计误差: %.2f°', mean(est_error)));
    % text(0.1, 0, sprintf('系统平均温度: %.1f℃', mean(temperature)));
    
    % figure
    % subplot 121
    % plot(NSV1)
    % hold on
    % plot(NSV1_used)
    % subplot 122
    % plot(NSV2)
    % hold on
    % plot(NSV2_used)
    % 保存图形
    % saveas(fig, 'auax_navigation_analysis.png');
end

%% 协议专用辅助函数
function distance = calculate_auax_distance(lon, lat)
    % 计算轨迹总距离 (Haversine公式)
    R = 6371000; % 地球半径(m)
    dlat = diff(deg2rad(lat));
    dlon = diff(deg2rad(lon));
    
    a = sin(dlat/2).^2 + cos(deg2rad(lat(1:end-1))) .* cos(deg2rad(lat(2:end))) .* sin(dlon/2).^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    distance = sum(R * c) / 1000; % 转换为km
end


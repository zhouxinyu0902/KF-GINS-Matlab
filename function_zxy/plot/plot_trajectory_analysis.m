function plot_trajectory_analysis(time, truth_pos, dr_pos, beacon_pos, period)
    % time: (n x 1), truth/dr/beacon_pos: (n x 3) [lat, lon, alt]
    
    deg2rad = pi / 180;
    n = length(time);
    dist_truth = zeros(n, 1);
    dist_dr = zeros(n, 1);

    % 1. 计算距离
    fprintf('正在计算距离数据...\n');
    for i = 1:n
        b_rad = [beacon_pos(i,1)*deg2rad,  beacon_pos(i,2)*deg2rad,  beacon_pos(i,3)];
        t_rad = [truth_pos(i,1)*deg2rad,  truth_pos(i,2)*deg2rad,  truth_pos(i,3)];
        d_rad = [dr_pos(i,1)*deg2rad,     dr_pos(i,2)*deg2rad,     dr_pos(i,3)];
        
        dist_truth(i) = RCompu(b_rad, t_rad);
        dist_dr(i)    = RCompu(b_rad, d_rad);
    end

    % 2. 轨迹连线图
    % figure('Color', 'w', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.5, 0.8]);
    myfigurestartup(12,12,'prese')
    subplot(2,2,[1,2]);
    hold on; grid on; box on;
    
    p1 = plot(truth_pos(:,2), truth_pos(:,1), 'k-', 'LineWidth', 1.5);
    p2 = plot(dr_pos(:,2), dr_pos(:,1), 'r-', 'LineWidth', 1.2);
    p3 = scatter(beacon_pos(1:period:n,2), beacon_pos(1:period:n,1), '.');

    % 连线逻辑
    for i = 1:period:n
        % 绘制关联线
        line([beacon_pos(i,2), truth_pos(i,2)], [beacon_pos(i,1), truth_pos(i,1)], ...
             'Color', [1 0.7 0.7], 'LineStyle', '-', 'LineWidth', 0.5);
        line([beacon_pos(i,2), dr_pos(i,2)], [beacon_pos(i,1), dr_pos(i,1)], ...
             'Color', [0 0.7 0.7], 'LineStyle', '-', 'LineWidth', 0.8);
        line([truth_pos(i,2), dr_pos(i,2)], [truth_pos(i,1), dr_pos(i,1)], 'LineStyle', '--', 'LineWidth', 0.8); 
        % text(beacon_pos(i,2), beacon_pos(i,1), [num2str(time(i)), 's'], 'FontSize', 7, 'Color', 'blue');
    end
    
    xlabel('经度 (deg)'); ylabel('纬度 (deg)');
    title(['轨迹对比图 (采样周期: ', num2str(period), 's)']);
    legend([p1, p2, p3], {'真实轨迹', '推算轨迹', '信标动态位置'}, 'Location', 'best');

    % 3. 距离变化图
    subplot(2,2,3);
    hold on; grid on;
    plot(time, dist_truth, 'k-', 'LineWidth', 1.2, 'DisplayName', '信标-真实');
    plot(time, dist_dr, 'r-', 'LineWidth', 1.2, 'DisplayName', '信标-推算');
    
    xlabel('时间 (s)'); ylabel('距离 (m)');
    title('信标观测距离分析');
    legend show;

    subplot(2,2,4);
    hold on; grid on;
    plot(time, dist_truth-dist_dr,'LineWidth', 1.2, 'DisplayName', '差值');
    legend show;
end
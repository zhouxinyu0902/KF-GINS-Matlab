function plot_trajectory_and_beacons_m(trajectory_xyz_m, local_beacons_m)
%PLOT_NAV_SCENE 绘制导航场景：轨迹、信标、起点和终点。
%
%   输入参数:
%   - trajectory_xyz_m: Nx3 矩阵，航行器轨迹在本地坐标系中的XYZ位移，单位米。
%   - local_beacons_m: Mx3 矩阵，M个信标在本地坐标系中的XYZ位移，单位米。

    %% 1. 参数设置与初始化
    num_beacons = size(local_beacons_m, 1);
    N = size(trajectory_xyz_m, 1);
    
    % 绘图样式定义
    beacon_style = {'p', 'MarkerSize', 10, 'LineWidth', 1.5, 'Color', [0 0.5 0]}; % 五角星(p)，绿色
    trajectory_style = {'b-', 'LineWidth', 2}; % 蓝色实线，更粗
    start_style = {'ro', 'MarkerSize', 8, 'LineWidth', 1.5}; % 红色圆圈
    end_style = {'ws', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k'}; % 黄色方块

    %% 2. 绘图：本地坐标系下的 2D 轨迹与信标 (X-Y 平面)
    
    myfigurestartup(7,5,'prese')
    
    hold on; % 保持图窗

    % --- 2.1 绘制信标位置 (背景图例) ---
    % 仅绘制一个点用于图例，然后关闭句柄可见性
    if num_beacons > 0
        plot(local_beacons_m(1, 1), local_beacons_m(1, 2), ...
             beacon_style{:}, ...
             'DisplayName', '锚点' );
    end

    % --- 2.2 绘制轨迹线 (平台轨迹) ---
    plot(trajectory_xyz_m(:, 1), trajectory_xyz_m(:, 2), ...
         trajectory_style{:}, ...
         'DisplayName', '轨迹');
     
    % --- 2.3 绘制起点和终点 (并调整图例顺序) ---
    % 绘制运动起点
    h_start = plot(trajectory_xyz_m(1, 1), trajectory_xyz_m(1, 2), ...
         start_style{:}, ... 
         'DisplayName', '运动起点');

    % 绘制运动终点
    h_end = plot(trajectory_xyz_m(N, 1), trajectory_xyz_m(N, 2), ...
         end_style{:}, ... 
         'DisplayName', '运动终点');
     
    % --- 2.4 绘制信标位置和标签 (文本和实际标记) ---
    for i = 1:num_beacons
        x = local_beacons_m(i, 1);
        y = local_beacons_m(i, 2);
        
        % 绘制信标标记，关闭句柄可见性
        plot(x, y, beacon_style{:}, 'HandleVisibility', 'off' ); 
        
        % 添加文本标签：信标号和坐标
        text(x + 10, y + 10, ... % 稍微偏移，避免覆盖标记
             sprintf('锚点 %d\n[%.0f, %.0f]', i, x, y), ...
             'VerticalAlignment', 'bottom', ...
             'HorizontalAlignment', 'left', ...
             'FontSize', 10, ...
             'Color', 'k');
    end

    % % --- 2.5 绘制方向箭头 (Direction Arrow) ---
    % 
    % % 箭头起点和方向    
    % % 绘制箭头
    % quiver(-50, -100, 0, 150, 0, ... % 0 表示不自动缩放箭头长度
    %        'MaxHeadSize', 0.5, 'LineWidth', 2, 'Color', 'k','HandleVisibility','off');
    % 
    % % % 2.6 添加起点、终点标签
    % % text(trajectory_xyz_m(1, 1) + 10, trajectory_xyz_m(1, 2) + 10, '起点', 'FontSize', 10, 'Color', 'k');
    % % text(trajectory_xyz_m(N, 1) - 60, trajectory_xyz_m(N, 2) - 10, '终点', 'FontSize', 10, 'Color', 'k');

    %% 3. 设置图表属性
    xlabel('东向 (m)');
    ylabel('北向 (m)');
    title('本地坐标系下的轨迹与锚点');
    grid on; 
    % axis equal; 
    
    % 设置坐标轴范围 (可选，如果需要固定视图)
    % xlim([-450, 250]);
    % ylim([-350, 250]);

    % 显示图例，并设置图例元素的顺序以匹配参考图
    % legend('show', 'Location', 'best'); 
    legend();
    hold off;
end
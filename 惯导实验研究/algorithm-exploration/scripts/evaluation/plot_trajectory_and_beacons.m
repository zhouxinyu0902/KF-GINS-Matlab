function fig = plot_trajectory_and_beacons(truthpath, beacon_pos)
% PLOT_TRAJECTORY_AND_BEACONS 绘制真实轨迹与信标的平面对比图 (地理坐标转局部平面坐标)
%
% 输入:
%   truthpath  - 真值文件路径 (第一列序号，第二列时间，第三列开始为 纬度(deg), 经度(deg), 高程(m) ...)
%   beacon_pos - 3x3 矩阵，每一行为一个信标的位置，单位为 [纬度(rad), 经度(rad), 高程(m)]
%
% 输出:
%   fig        - 图形句柄

    %% 1. 参数检查
    if nargin < 2
        error('必须提供真值文件路径和 3x3 的信标位置矩阵');
    end
    if ~isequal(size(beacon_pos), [3, 3])
        error('信标位置矩阵 beacon_pos 必须是 3x3 的矩阵');
    end

    %% 2. 读取并解析真值数据
    try
        temp = importdata(truthpath);
        truth_data = temp(:, 2:end); % 剔除第一列序号
    catch
        error('无法读取真值文件: %s', truthpath);
    end
    
    % 提取轨迹的 纬度、经度、高程 (真值文件中通常前三列为 Lat(deg), Lon(deg), H(m))
    traj_lat = truth_data(:, 2)*pi/180;
    traj_lon = truth_data(:, 3)*pi/180;
    traj_h = truth_data(:, 4);
    glvs 
    trj_xyz = pos2dxyz([traj_lat,traj_lon,traj_h],[traj_lat(1);traj_lon(1);traj_h(1)]);
    bea_trj = pos2dxyz(beacon_pos,[traj_lat(1);traj_lon(1);traj_h(1)]);
    traj_east_km = trj_xyz(:,1)/1000;
    traj_north_km = trj_xyz(:,2)/1000;
    beacon_east_km = bea_trj(:,1)/1000;
    beacon_north_km = bea_trj(:,2)/1000;
    %% 6. 绘制平面对比图

    fig = myfigurestartup(7, 5, 'prese'); % 保持合理的宽高比
    hold on; grid on; box on;

    % 1. 画真实轨迹
    plot(traj_east_km, traj_north_km, '-', 'Color', [0.1, 0.5, 0.8],'DisplayName', '真实轨迹');
    
    % 2. 画轨迹起点
    plot(0, 0, 'go', 'MarkerFaceColor', 'g', 'DisplayName', '轨迹起点');

    % 3. 画信标位置 (使用五角星单独标出)
    scatter(beacon_east_km, beacon_north_km, 150, 'r', 'p', 'Filled', 'DisplayName', '信标');
    
    % 4. 为信标添加文本标签 (信标 1, 2, 3)
    for i = 1:3
        text(beacon_east_km(i), beacon_north_km(i), sprintf('  信标 %d', i), 'FontWeight', 'bold', 'Color', 'r', 'VerticalAlignment', 'middle');
    end

    %% 7. 图形美化
    xlabel('东向距离 East (km)');
    ylabel('北向距离 North (km)');
    title('真实轨迹与信标平面位置对比图', 'FontWeight', 'bold');
    
    % 强制 1:1 坐标轴比例，防止地图拉伸变形
    axis equal; 
    
    % 稍微放大边距，防止信标或文字压线
    margin = 0.1 * max([max(abs(traj_east_km)), max(abs(beacon_east_km))]);
    xlim([min([traj_east_km; beacon_east_km]) - margin, max([traj_east_km; beacon_east_km]) + margin]);
    ylim([min([traj_north_km; beacon_north_km]) - margin, max([traj_north_km; beacon_north_km]) + margin]);
    
    legend('show', 'Location', 'bestoutside');
    set(findall(fig, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
end

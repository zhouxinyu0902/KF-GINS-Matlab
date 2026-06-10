function rangedataget(path, align_mode)
    % =========================================================================
    % 函数名称：rangedataget
    % 功能描述：基于GNSS基准轨迹，反向仿真生成3个地面无线电/伪卫星信标的测距解算数据
    % 输入参数:
    %   path       - 数据及配置文件存放的文件夹路径 (String)
    %   align_mode - 阵列基准对齐模式 (String)
    %                'right'  : 以等边三角形的【右侧斜边】中点作为车辆轨迹起点
    %                'bottom' : 以等边三角形的【平坦底边】中点作为车辆轨迹起点
    % =========================================================================
    
    % 缺省参数容错处理
    if nargin < 2
        align_mode = 'right'; 
    end

    % 1. 安全读取标准参考轨迹 (Truth Nav)
    truth_file = fullfile(path, 'truth.nav');
    if ~exist(truth_file, 'file')
        error('错误：未在路径 %s 下找到参考轨迹 truth.nav 文件！', path);
    end
    truth = importdata(truth_file);
    GNSS_1s = truth(1:100:end, 2:5); % 降采样至 1Hz 提取 [时间, 纬度, 经度, 高程]
    
    %% 2. 阵列几何拓扑定义与空间变换
    glvs;
    orgin0 = d2r(GNSS_1s(1, 2:4)); % 提取轨迹首帧作为站心局部导航坐标系(ENU)原点
    
    % --- 【修正定义】标准等边三角形阵列坐标（单位：米，基线长度 20km） ---
    % 点1: 上方顶点; 点2: 左下角顶点; 点3: 右下角顶点
    % 此时 点2-点3 构成了绝对意义上的水平【底边】，点1-点3 构成了【右侧边】
    dxyz_original = [  0,  5*sqrt(3), 0;
                     -10, -5*sqrt(3), 0;
                      10, -5*sqrt(3), 0] * 1000;
                  
    % 配置阵列整体旋转角度 (当前设定 0 deg)
    theta_deg = 0;
    theta_rad = deg2rad(theta_deg);
    
    % 构造绕 Z 轴（天向轴）自转的旋转矩阵
    R = [cos(theta_rad), -sin(theta_rad), 0;
         sin(theta_rad),  cos(theta_rad), 0;
                      0,               0, 1];
                  
    % 步骤 I：阵列绕自身几何中心进行自转
    center_xyz = mean(dxyz_original, 1);               
    dxyz_centered = dxyz_original - center_xyz;        
    dxyz_rotated = (R * dxyz_centered')' + center_xyz; 
    
    % 步骤 II：根据所选方案，精确定位对准图钉的“边中点”
    if strcmpi(align_mode, 'right')
        % 方案 A：右侧斜边中点 (点1与点3的连线中点)
        target_midpoint = (dxyz_rotated(1, :) + dxyz_rotated(3, :)) / 2;
        fprintf('🎯 [阵列对齐] 当前配置: 以等边三角形【右侧斜边】中点对齐车辆起点。\n');
        
    elseif strcmpi(align_mode, 'bottom')
        % 方案 B：平坦底边中点 (点2与点3的连线中点)
        target_midpoint = (dxyz_rotated(2, :) + dxyz_rotated(3, :)) / 2;
        fprintf('🎯 [阵列对齐] 当前配置: 以等边三角形【水平底边】中点对齐车辆起点。\n');
        
    else
        error('错误：未知的对齐模式！请输入 ''right'' 或 ''bottom''。');
    end
    
    % 步骤 III：阵列整体刚性平移，将选定的边中点重合至 ENU 坐标系原点 (0,0,0)
    dxyz_final = dxyz_rotated - target_midpoint; 
    
    %% 3. 导航系坐标系重构转换
    rrm = dxyz2pos(dxyz_final, orgin0'); % 局部局部米级坐标转大椭球大地球经纬高
    beaconxyz = dxyz_final(1:3, :);
    beaconrrm = rrm(1:3, :);
    
    % 转化动态轨迹至本地 ENU 坐标系
    trj = GNSS_1s(:, 2:4);
    trj(:, 1:2) = d2r(trj(:, 1:2));
    trajectory_xyz = pos2dxyz(trj, orgin0'); 
    
    % 绘图模块 1：拓扑展板（动态观察轨迹是否完美从设定的边中点出发）
    plot_trajectory_and_beacons_m(trajectory_xyz, beaconxyz);
    
    %% 4. 高精度测距仿真解算
    trajectory_x = trajectory_xyz(:, 1);
    trajectory_y = trajectory_xyz(:, 2);
    time_s = GNSS_1s(:, 1);
    num_points = length(time_s);
    
    distances = zeros(num_points, 3);
    
    % 绘图模块 2：动态测距收敛观测曲线
    figure('Name', sprintf('信标解算距离趋势 (%s)', align_mode), 'Color', 'w');
    hold on; grid on;
    plot_colors = {'#D95319', '#77AC30', '#0072BD'}; % 规范工程配色：橙、绿、蓝
    
    for i = 1:3
        % 计算运载体到各个地面静态信标的空间 2D 基线平距
        distances(:, i) = sqrt((trajectory_x - beaconxyz(i, 1)).^2 + ...
                               (trajectory_y - beaconxyz(i, 2)).^2);
                           
        plot(time_s, distances(:, i)/1000, 'LineWidth', 1.5, 'Color', plot_colors{i}, ...
             'DisplayName', sprintf('至地面信标 Beacon %d', i)); 
    end
    xlabel('GPS时间 (s)', 'FontSize', 11);
    ylabel('空间几何距离 (km)', 'FontSize', 11);
    title(sprintf('运载体至静态信标 2D 测距时历曲线 (%s 模式)', align_mode), 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');
    
    % 仿真传感器高斯白噪声（如需更贴近半实物仿真环境，可取消下行注释）
    % distances = distances + normrnd(0, 15, size(distances));
    
    %% 5. 格式化数据矩阵批量写盘
    for i = 1:3
        % 平铺信标的大地坐标 [Lat, Lon, Alt] 为 N 行 3 列矩阵
        beacon_ext = repmat(beaconrrm(i, :), num_points, 1);
        
        % 按照紧凑型量测更新矩阵组装：[时间, 距离, 距离(双向测距占位), 信标Lat, 信标Lon, 信标Alt]
        range_matrix = [time_s, distances(:, i), distances(:, i), beacon_ext];
        
        output_file = fullfile(path, sprintf('range%d.txt', i));
        
        try
            writematrix(range_matrix, output_file, 'Delimiter', ' ');
            fprintf('  [导出成功] 成功生成测距量测文件: range%d.txt\n', i);
        catch ME
            warning('  [导出失败] 无法写入文件 range%d.txt. 错误描述: %s', i, ME.message);
        end
    end
end
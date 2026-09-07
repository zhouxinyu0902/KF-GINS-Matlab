function [beacon_data, moving_beacons,moving_beacons1] = beacon_gen_v2(rngk, avp_true, num_fixed, deltaT, isfig)
    % -------------------------------------------------------------------------
    % 优化说明：
    % 1. 向量化处理：减少 for 循环，提升大数据量下的生成速度。
    % 2. 坐标转换优化：统一参考系，确保 pos_ref 格式严谨。
    % 3. 绘图增强：增加起点标注与颜色区分。
    % -------------------------------------------------------------------------
    glvs;
    pos_true = avp_true(:, 7:9); 
    pos_ref  = avp_true(1, 7:9)'; % [lat; lon; hgt]
    N = size(pos_true, 1);
    
    %% 1. 固定信标逻辑 (向量化计算位置)
    % dpos_fixed_ned = [
    %     1310, -930, 0; 140, 960, 0; -900, -930, 0;
    %     900, 960, 0; -1834, -570, 0; -70, -2520, 0] / 3;
    % dpos_fixed_ned = [
    %     -300, -400, 0; 
    %     -3000, 4000, 0; 
    %     -300, -1000, 0;
    %     1000, -400, 0; 
    %     1000, 400, 0; 
    %     1000, -1000, 0;
    %     2200, -400, 0; 
    %     2200, 400, 0; 
    %     2200, -1000, 0] ;
    %% 竖向的信标设置
    % dpos_fixed_ned = [
    %     0, 4000, 0; 
    %     -3000, 4000, 0; 
    %     -3000, -2000, 0;
    %     0, 400, 0;
    %     -300, 400, 0; 
    %     -300, -200, 0;
    %     3000, 4000, 0; 
    %     3000, -2000, 0;];
    %% 横向的信标设置
    % dpos_fixed_ned = [
    %     0, 4000, 0; 
    %     -3000, 4000, 0; 
    %     -3000, -2000, 0;
    %     0, 400, 0;
    %     -300, 400, 0; 
    %     -300, -200, 0;
    %     3000, 4000, 0; 
    %     3000, -2000, 0;];
    %% 近场信标
    dpos_fixed_ned=[
        400,400,0;
        400,0,0;
        400,-400,0;
        0,400,0;
        0,-400,0;
        -400,400,0;
        -400,0,0;
        -400,-400,0
        ];
    %% 远场
    dpos_fixed_ned=[
        400,400,0;
        % 400,0,0;
        400,-400,0;
        % 0,400,0;
        % 0,-400,0;
        -400,400,0;
        % -400,0,0;
        -400,-400,0
        ]*20;
    % dpos_fixed_ned=[
    %     400,800,0;
    %     % 400,0,0;
    %     400,-400,0;
    %     % 0,400,0;
    %     % 0,-400,0;
    %     -800,800,0;
    %     % -400,0,0;
    %     -800,-400,0
    %     ]*2;

    dpos_fixed_ned=[
        -1000.48,1311.9,0;
        968.138,134.34,0;
        -563.946,-1830.64,0;
        -2534.58,-70.34,0
        ];

    
    %%
    dpos_fixed_ned(:,1:2)=dpos_fixed_ned(:,[2,1]);
        

    num_fixed = min(num_fixed, size(dpos_fixed_ned, 1));
    dpos_fixed_ned = dpos_fixed_ned(1:num_fixed, :);
    
    % 一次性计算所有固定信标的 LLH
    beacon_fixed_llh = zeros(num_fixed, 3);
    for i = 1:num_fixed
        beacon_fixed_llh(i, :) = dxyz2pos(dpos_fixed_ned(i, :), pos_ref);
    end
    % 向量化计算距离 (依赖 RCompu 支持矩阵输入)
    % range_fixed: [N x num_fixed]
    range_fixed = zeros(N, num_fixed);
    range_fixed_true = zeros(N, num_fixed);
    for i = 1:num_fixed
        rng(111)
        dist_true = RCompu(pos_true, beacon_fixed_llh(i, :));
        range_fixed(:, i) = dist_true + randn(N, 1) * rngk;
        range_fixed_true(:, i) = dist_true;
    end

    %% 2. 移动信标逻辑 (结构化重构)
    idx_down = 1:deltaT:N;
    pos_v_down = pos_true(idx_down, :);
    N_down = length(idx_down);
    
    % 获取移动信标轨迹
    [E_m, N_m] = llh_simu(N_down); 
    dpos_m_ned = [N_m', E_m', zeros(N_down, 1)]; 
    
    % 坐标转换
    beacon_m_llh = zeros(N_down, 3);
    for j = 1:N_down
        beacon_m_llh(j, :) = dxyz2pos(dpos_m_ned(j, :), pos_ref);
    end
    
    % 距离计算与数值保护
    slant_m = RCompu(pos_v_down, beacon_m_llh);
    dhgt_m = pos_v_down(:, 3) - beacon_m_llh(:, 3);
    % 避免由于噪声导致 sqrt 负值
    hori_r_true = sqrt(max(slant_m.^2 - dhgt_m.^2, 0));
    range_m_noisy = hori_r_true + randn(N_down, 1) * rngk;
    range_m = hori_r_true;
    %%
    dllh = load('D:\Github\PSINS\psins2401\mytest\03_sum\data_1\dllh.mat');
    dpos_m_ned1 = [dllh.dllh(:,[2,1])*glv.Re, zeros(1101, 1)]; 
    % 坐标转换

    beacon_m_llh1 = zeros(length(dpos_m_ned1), 3);
    for j = 1:length(dpos_m_ned1)
        beacon_m_llh1(j, :) = dxyz2pos(dpos_m_ned1(j, :), pos_v_down(16*(j-1)+1, :)');
    end
    
    % 距离计算与数值保护
    slant_m = RCompu(pos_v_down(1:16:end, :), beacon_m_llh1);
    dhgt_m = pos_v_down(1:16:end, 3) - beacon_m_llh1(:, 3);
    % 避免由于噪声导致 sqrt 负值
    hori_r_true1 = sqrt(max(slant_m.^2 - dhgt_m.^2, 0));
    range_m_noisy1 = hori_r_true1 + randn(length(dpos_m_ned1), 1) * rngk;
    range_m1 = hori_r_true1;



    %% 3. 输出打包
    beacon_data.fixed_pos   = beacon_fixed_llh;
    beacon_data.fixed_range = range_fixed;
    beacon_data.fixed_range_true = range_fixed_true;
    moving_beacons.pos      = beacon_m_llh;
    moving_beacons.range    = range_m_noisy;
    moving_beacons.range_true    = range_m;
    moving_beacons.time_idx = idx_down;
    moving_beacons.type     = 'Horizontal';
    

    moving_beacons1.pos      = beacon_m_llh1;
    moving_beacons1.range    = range_m_noisy1;
    moving_beacons1.range_true    = range_m1;
    moving_beacons1.time_idx = idx_down;
    moving_beacons1.type     = 'Horizontal';
    %% 4. 可视化优化
    if isfig
        pos_true_xyz = pos2dxyz(pos_true,pos_true(1,:)');
        beacon_fixed_xyz = pos2dxyz(beacon_fixed_llh,pos_true(1,:)');
        beacon_m_xyz = pos2dxyz(beacon_m_llh,pos_true(1,:)');

        beacon_m_xyz1 = pos2dxyz(beacon_m_llh1,pos_true(1,:)');
        myfigurestartup(12,7,'prese');
        % 1. 载体轨迹
        plot(pos_true_xyz(:, 1),pos_true_xyz(:, 2), 'k', 'LineWidth', 1.5, 'DisplayName', 'Vehicle Path');
        hold on; grid on;
        
        % 2. 固定信标 (使用不同的颜色和标注)
        scatter(beacon_fixed_xyz(:, 1), beacon_fixed_xyz(:, 2), 60, 'r^', 'filled', 'DisplayName', 'Fixed Beacons');
        for i = 1:num_fixed
            text(beacon_fixed_xyz(i, 1), beacon_fixed_xyz(i, 2), sprintf(' F%d', i), 'Color', 'r');
        end
        
        % 3. 移动信标
        plot(beacon_m_xyz(:, 1), beacon_m_xyz(:, 2), 'g--', 'LineWidth', 1.2, 'DisplayName', 'Moving Beacon 1');
        plot(beacon_m_xyz(1, 1), beacon_m_xyz(1, 2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'M-Start');
        

        plot(beacon_m_xyz1(:, 1), beacon_m_xyz1(:, 2), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Moving Beacon 2');
        plot(beacon_m_xyz1(1, 1), beacon_m_xyz1(1, 2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'M-Start');

        xlabel('Longitude (m)'); ylabel('Latitude (m)');
        title('Navigation System Beacon & Trajectory');
        legend('Location', 'northeastoutside'); 
        axis equal;

    end
end
function [E_traj, N_traj]=llh_simu(len)
% 红色起点 (Start) 附近的起始区域: ~(-250, -50)
% waypoints_E = [-250, -200, -400, -700, -600, -300, 200, 800, 1000, 700, 200, 0, -200, -250]*1.5;
% waypoints_N = [-50, 0, 500, 450, 50, -200, -700, -900, -700, -500, -300, -100, -50, -50]*1.5;
waypoints_N = [500, 400, 1000, 1200, 1800, 2100, 1900, 1500, 1100, 800, 500, 300, 100];
waypoints_E = [200, 100, 50, -200, -500, -900, -1300, -1400, -1200, -1000, -800, -600, -300];


waypoints_N = [100, 300, 600, 900, 1100, 1000, 700, 400, 100, -100, 100, 400, 700];
waypoints_E = [100, -150, -400, -700, -1000, -1300, -1450, -1300, -1000, -700, -400, -100, 100];
% 2. 定义时间点

T_total = 9000;
t_waypoints = linspace(0, T_total, length(waypoints_E));
% 3. 生成平滑的轨迹点
% 使用插值生成 1000 个平滑点 (t_fine)
t_fine = linspace(0, T_total, len); 
% 使用 PCHIP (分段三次 Hermite 插值) 保持形状的局部特征
E_traj = interp1(t_waypoints, waypoints_E, t_fine, 'pchip');
N_traj = interp1(t_waypoints, waypoints_N, t_fine, 'pchip');
end
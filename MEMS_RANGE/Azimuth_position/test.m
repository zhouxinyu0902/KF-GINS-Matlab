clc; clear; format long g;

%% --- 1. 定义测试场景 ---
% 场景 A: 之前的经典测试点 (预期角度 -135度)
test_cases(1).name = '场景A (东北向航行，目标在正北)';
test_cases(1).my_pos_true = [100.0, 90.0];   % [East, North]
test_cases(1).beacon_pos  = [100.0, 100.0];  % [East, North]
test_cases(1).heading     = 45.0;            % 航向

% 场景 B: 随机生成一个复杂场景
test_cases(2).name = '场景B (随机复杂坐标)';
test_cases(2).my_pos_true = [523.5, -120.4];
test_cases(2).beacon_pos  = [-40.2, 88.9];
test_cases(2).heading     = 192.5;

%% --- 2. 循环验证 ---
fprintf('======================================================\n');
fprintf('                函数闭环一致性验证\n');
fprintf('======================================================\n');

for i = 1:length(test_cases)
    t = test_cases(i);
    fprintf('\n>>> 测试: %s\n', t.name);
    
    % [步骤1: 准备数据]
    % 计算真实的几何距离 (Range)
    dist_vec = t.beacon_pos - t.my_pos_true; 
    true_range = norm(dist_vec); 
    
    % [步骤2: 正向计算 (Pos -> Angle)]
    % 计算方位角 theta
    calc_theta = pos2azimuth(t.my_pos_true, t.beacon_pos, t.heading);
    
    % [步骤3: 反向计算 (Angle + Range -> Pos)]
    % 尝试还原位置
    calc_my_pos = calc_position_from_beacon(t.beacon_pos, true_range, calc_theta, t.heading);
    
    % [步骤4: 误差分析]
    pos_error = norm(t.my_pos_true - calc_my_pos);
    
    % 输出详情
    fprintf('   原始位置 (E, N): [%.4f, %.4f]\n', t.my_pos_true(1), t.my_pos_true(2));
    fprintf('   计算角度 (Theta): %.4f 度\n', calc_theta);
    fprintf('   反算位置 (E, N): [%.4f, %.4f]\n', calc_my_pos(1), calc_my_pos(2));
    
    if pos_error < 1e-9
        fprintf('   [结果]: ✅ 验证通过 (误差: %.2e)\n', pos_error);
    else
        fprintf('   [结果]: ❌ 验证失败 (误差过大: %.4f)\n', pos_error);
    end
end

fprintf('\n======================================================\n');

%% --- 附录：你的两个函数 (保持原样) ---

function my_pos = calc_position_from_beacon(beacon_xy, range, theta_deg, heading_deg)
    % 1. 还原相对于船首的角度
    rel_bearing_to_bow = theta_deg + 90;
    % 2. 计算全局真方位角
    true_bearing_deg = heading_deg + rel_bearing_to_bow;
    true_bearing_rad = deg2rad(true_bearing_deg);
    % 3. 计算位移向量
    delta_E = range * sin(true_bearing_rad);
    delta_N = range * cos(true_bearing_rad);
    % 4. 反向解算位置
    my_E = beacon_xy(1) - delta_E;
    my_N = beacon_xy(2) - delta_N;
    my_pos = [my_E, my_N];
end

function theta_to_normal = pos2azimuth(dr_xy, beacon_xy, dr_heading_deg)
    % 1. 提取坐标
    beacon_E = beacon_xy(1);
    beacon_N = beacon_xy(2);
    dr_E = dr_xy(1);
    dr_N = dr_xy(2);
    % 2. 计算差分
    delta_E = beacon_E - dr_E;
    delta_N = beacon_N - dr_N;
    % 3. 计算全局真方位角
    true_bearing = rad2deg(atan2(delta_E, delta_N)); 
    % 4. 减去航向角
    rel_bearing_bow = true_bearing - dr_heading_deg;
    % 5. 减去90度 (基线法线指向右舷)
    raw_theta = rel_bearing_bow - 90; 
    % 6. 归一化
    theta_to_normal = mod(raw_theta + 180, 360) - 180;
end
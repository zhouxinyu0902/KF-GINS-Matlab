function params = analyze_mems_engineering_units(data, fs, lat_deg, Cbn)
    % data: [N x 6] 角增量(rad), 速度增量(m/s)
    % fs: 采样频率 (Hz)
    % Cbn: 姿态矩阵 (b到n)

    dt = 1 / fs;
    
    % --- 1. 环境参数与投影 (SI单位) ---
    we = 7.2921151467e-5; 
    L = deg2rad(lat_deg);
    g_val = 9.780327 * (1 + 0.0053024*sin(L)^2 - 0.0000058*sin(2*L)^2);
    
    w_ie_b = Cbn' * [we * cos(L); 0; -we * sin(L)];
    g_b = Cbn' * [0; 0; -g_val];

    % --- 2. 提取统计量 (SI单位) ---
    gyro_rate = data(:, 1:3) / dt; 
    acc_rate = data(:, 4:6) / dt;

    bias_g_si = mean(gyro_rate) - w_ie_b';
    bias_a_si = mean(acc_rate) - g_b';
    
    std_g_si = std(gyro_rate);
    std_a_si = std(acc_rate);

    % --- 3. 换算为指定工程单位 ---
    
    % Gyro Bias: rad/s -> deg/h
    params.gyrobias_degh = rad2deg(bias_g_si) * 3600;
    
    % Gyro ARW: (rad/s)/sqrt(Hz) -> deg/sqrt(h)
    % 公式: ARW_deg_sqrt_h = std_rad_s / sqrt(fs) * (180/pi) * 60
    params.gyroarw_degsqh = (std_g_si / sqrt(fs)) * (180/pi) * 60;
    
    % Acc Bias: m/s^2 -> mGal (1 mGal = 1e-5 m/s^2)
    params.accbias_mGal = bias_a_si * 1e5;
    
    % Acc VRW: (m/s^2)/sqrt(Hz) -> m/s/sqrt(h)
    % 公式: VRW_m_s_sqrt_h = std_m_s2 / sqrt(fs) * 60
    params.accvrw_ms_sqh = (std_a_si / sqrt(fs)) * 60;

    % --- 打印结果供比对 ---
    fprintf('--- 仿真参数校验 (工程单位) ---\n');
    fprintf('Gyro Bias: [%.4f, %.4f, %.4f] deg/h\n', params.gyrobias_degh);
    fprintf('Gyro ARW:  [%.4e, %.4e, %.4e] deg/sqrt(h)\n', params.gyroarw_degsqh);
    fprintf('Acc Bias:  [%.4f, %.4f, %.4f] mGal\n', params.accbias_mGal);
    fprintf('Acc VRW:   [%.4e, %.4e, %.4e] m/s/sqrt(h)\n', params.accvrw_ms_sqh);
end


%% 严格修正的向量化解析函数
function IMU_DATA = parse_rawimub_strict(allData, headerPositions)
    % --- 1. 参数定义 (严格对应原函数) ---
    H = 28; % 帧头长度
    packetSize = H + 45; % 总包长 73字节
    G_VAL = 9.806; 
    
    % 比例因子
    ACC_SCALE_FACTOR = G_VAL * 655360;
    GYRO_SCALE_FACTOR = 160849.543863;

    % --- 2. 边界检查 ---
    % 移除末尾不完整的包
    if ~isempty(headerPositions) && (headerPositions(end) + packetSize - 1 > length(allData))
        headerPositions(end) = []; 
    end
    
    numPackets = length(headerPositions);
    if numPackets == 0
        IMU_DATA = [];
        return;
    end

    % --- 3. 核心修正：构建索引矩阵 ---
    % MATLAB 索引规则：如果 dataPacket 是从 1 开始的
    % 那么 dataPacket(K) 在 allData 中的绝对位置是：headerPos + K - 1
    
    % 为了速度，我们利用隐式扩展一次性计算所有索引
    % result_indices 维度为 [字节数 x 包数]
    
    % 辅助匿名函数：根据原代码中的 StartIndex 生成绝对索引矩阵
    % origin_start: 原代码中的起始位置 (例如 H+17)
    % byte_len: 数据长度 (例如 4)
    % 返回: 4xN 的索引矩阵
    get_idx = @(origin_start, byte_len) ...
        (origin_start - 1 + (0 : byte_len-1))' + headerPositions;

    % --- 4. 提取原始字节 (Raw Bytes) ---
    
    % 4.1 GNSS周数 (H+1 到 H+4) -> uint32
    idx_week = get_idx(H+1, 4);
    week_vec = double(typecast(reshape(allData(idx_week), [], 1), 'uint32'));
    
    % 4.2 GNSS周内秒 (H+5 到 H+12) -> double
    idx_second = get_idx(H+5, 8);
    second_vec = typecast(reshape(allData(idx_second), [], 1), 'double');
    
    % 4.3 IMU状态字 (H+15 到 H+16, 前补00 00) -> uint32
    % 原逻辑: typecast([0;0;dataPacket(H+15:H+16)], 'uint32')
    % 我们只提取后两个字节
    idx_status_bytes = get_idx(H+15, 2); 
    raw_status_bytes = allData(idx_status_bytes); % 2 x N 矩阵
    % 手动构造 [0; 0; byte1; byte2] 结构
    % 注意：MATLAB 列优先，reshape 后每4个字节组成一个数
    status_full_bytes = [zeros(2, numPackets, 'uint8'); raw_status_bytes];
    status_vec = double(typecast(status_full_bytes(:), 'uint32'));
    
    % 4.4 加速度 (int32)
    % Z轴 (H+17 到 H+20)
    idx_az = get_idx(H+17, 4);
    accel_z_vec = double(typecast(reshape(allData(idx_az), [], 1), 'int32')) / ACC_SCALE_FACTOR;
    
    % Y轴 (H+21 到 H+24)
    idx_ay = get_idx(H+21, 4);
    accel_y_vec = double(typecast(reshape(allData(idx_ay), [], 1), 'int32')) / ACC_SCALE_FACTOR;
    
    % X轴 (H+25 到 H+28)
    idx_ax = get_idx(H+25, 4);
    accel_x_vec = double(typecast(reshape(allData(idx_ax), [], 1), 'int32')) / ACC_SCALE_FACTOR;
    
    % 4.5 角速度 (int32)
    % Z轴 (H+29 到 H+32)
    idx_gz = get_idx(H+29, 4);
    gyro_z_vec = double(typecast(reshape(allData(idx_gz), [], 1), 'int32')) / GYRO_SCALE_FACTOR;
    
    % Y轴 (H+33 到 H+36)
    idx_gy = get_idx(H+33, 4);
    gyro_y_vec = double(typecast(reshape(allData(idx_gy), [], 1), 'int32')) / GYRO_SCALE_FACTOR;
    
    % X轴 (H+37 到 H+40)
    idx_gx = get_idx(H+37, 4);
    gyro_x_vec = double(typecast(reshape(allData(idx_gx), [], 1), 'int32')) / GYRO_SCALE_FACTOR;

    % --- 5. 组装结果 ---
    % 严格遵守您原代码的输出顺序:
    % IMU_DATA(i, :) = [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, second, week, status]
    
    IMU_DATA = [gyro_x_vec, gyro_y_vec, gyro_z_vec, ...
                accel_x_vec, accel_y_vec, accel_z_vec, ...
                second_vec, week_vec, status_vec];
            
    % 打印统计信息 (可选)
    fprintf('成功解析 %d 个数据包\n', numPackets);
end
% 从文件读取并解析RAWIMUB数据包，只处理状态字为00的正常数据
clear
% filename = 'C:\Users\23764\Desktop\惯导试验\数据\SAVE2025_9_27_10-17-28.DAT'; % 替换为文件名
filename = 'C:\Users\23764\Desktop\惯导试验\数据\101500_4752450.dat'; % 替换为文件名


% 以二进制模式读取整个文件
fid = fopen(filename, 'rb');
allData = fread(fid, inf, 'uint8');
fclose(fid);

% 查找满足帧头条件的位置（28字节帧头）
headerPositions = [];
frameHeader = [170, 68, 18, 28, 12, 1, 0, 160]; % 帧头前8字节: AA 44 12 1C 0C 01 00 A0

% 查找帧头位置
for i = 1:length(allData)-72 % 需要至少73字节才能构成完整数据包 (H+45=28+45=73)
    % 检查前8个字节是否符合帧头特征
    if all(allData(i:i+7) == frameHeader')
        headerPositions = [headerPositions, i];
    end
end

% 初始化存储数组
ki = 1;
week_data = [];          % GNSS周数 (字节29-32)
second_data = [];        % GNSS周内秒 (字节33-40)
status_words = [];       % 状态字 (字节41-44) - Hex Ulong类型
accel_z = [];            % Z轴加速度 (字节45-48)
accel_y = [];            % Y轴加速度 (字节49-52)
accel_x = [];            % X轴加速度 (字节53-56)
gyro_z = [];             % Z轴角速度 (字节57-60)
gyro_y = [];             % Y轴角速度 (字节61-64)
gyro_x = [];             % X轴角速度 (字节65-68)
crc_values = [];         % CRC32校验值 (字节69-72)

% 用于统计的变量
total_packets = length(headerPositions);
normal_packets = 0;
abnormal_packets = 0;

fprintf('开始解析RAWIMUB数据包...\n');
fprintf('找到 %d 个数据包\n', total_packets);

% 创建IMU_DATA矩阵存储结果
IMU_DATA = [];

% 定义H值（帧头长度）
H = 28;

% 比例系数参数
G_VAL = 9.806; % 重力加速度

for i = 1:length(headerPositions)
    startIdx = headerPositions(i);
    endIdx = startIdx + 72; % 73字节数据包
    
    if endIdx <= length(allData)
        dataPacket = allData(startIdx:endIdx);
        
        try
            % 验证帧头（前8字节）
            if all(dataPacket(1:8) == frameHeader')
                % 解析状态字 (字节41-44, Hex Ulong类型)
                % 注意：Hex Ulong就是uint32类型，但以十六进制格式显示
                status_val = typecast(uint8(dataPacket(41:44)), 'uint32');
                
                % % 只处理状态字为0x00（正常）的数据包
                % if status_val == 0
                    normal_packets = normal_packets + 1;
                    
                    % 解析GNSS周数 (字节29-32, Ulong类型)
                    week_val = typecast(uint8(dataPacket(29:32)), 'uint32');
                    week_data(ki, 1) = double(week_val);
                    
                    % 解析GNSS周内秒 (字节33-40, Double类型)
                    second_val = typecast(uint8(dataPacket(33:40)), 'double');
                    second_data(ki, 1) = second_val;
                    
                    % 存储状态字（Hex Ulong格式）
                    status_words(ki, 1) = status_val;
                    
                    % 解析加速度数据 (Long类型，有符号32位整数)
                    % Z轴加速度 (字节45-48)
                    accel_z_raw = typecast(uint8(dataPacket(45:48)), 'int32');
                    % 转换为物理量：原始值 / (G_VAL × 655360)
                    accel_z(ki, 1) = double(accel_z_raw) / (G_VAL * 655360)*100;
                    
                    % Y轴加速度 (字节49-52)
                    accel_y_raw = typecast(uint8(dataPacket(49:52)), 'int32');
                    accel_y(ki, 1) = double(accel_y_raw) / (G_VAL * 655360)*100;
                    
                    % X轴加速度 (字节53-56)
                    accel_x_raw = typecast(uint8(dataPacket(53:56)), 'int32');
                    accel_x(ki, 1) = double(accel_x_raw) / (G_VAL * 655360)*100;
                    
                    % 解析角速度数据 (Long类型，有符号32位整数)
                    % Z轴角速度 (字节57-60)
                    gyro_z_raw = typecast(uint8(dataPacket(57:60)), 'int32');
                    % 转换为物理量：原始值 / 160849.543863
                    gyro_z(ki, 1) = double(gyro_z_raw) / 160849.543863*100;
                    
                    % Y轴角速度 (字节61-64)
                    gyro_y_raw = typecast(uint8(dataPacket(61:64)), 'int32');
                    gyro_y(ki, 1) = double(gyro_y_raw) / 160849.543863*100;
                    
                    % X轴角速度 (字节65-68)
                    gyro_x_raw = typecast(uint8(dataPacket(65:68)), 'int32');
                    gyro_x(ki, 1) = double(gyro_x_raw) / 160849.543863*100;
                    
                    % 解析CRC32 (字节69-72, Hex类型)
                    crc_val = typecast(uint8(dataPacket(69:72)), 'uint32');
                    crc_values(ki, 1) = crc_val;
                    
                    % 存储到IMU_DATA矩阵
                    % 格式: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, week, second, status]
                    IMU_DATA(ki, 1:9) = [accel_x(ki), accel_y(ki), accel_z(ki), ...
                                         gyro_x(ki), gyro_y(ki), gyro_z(ki), ...
                                         double(week_val), second_val, double(status_val)];
                    
                    ki = ki + 1;
                % else
                %     abnormal_packets = abnormal_packets + 1;
                %     % 可选：记录异常状态字的值
                %     fprintf('数据包 %d: 状态字异常 (0x%08X)\n', i, status_val);
                % end
            else
                fprintf('数据包 %d: 帧头验证失败\n', i);
            end
            
        catch ME
            fprintf('解析数据包 %d (位置: %d) 时出错: %s\n', i, startIdx, ME.message);
        end
    end
end

% 显示解析结果统计
fprintf('\n===== 解析结果统计 =====\n');
fprintf('总数据包数量: %d 个\n', total_packets);
fprintf('正常数据包 (状态字=0x00): %d 个\n', normal_packets);
fprintf('异常数据包 (状态字≠0x00): %d 个\n', abnormal_packets);
fprintf('正常数据比例: %.2f%%\n', normal_packets/total_packets*100);

if normal_packets > 0
    % 显示GNSS信息
    fprintf('GNSS周数范围: %d 到 %d 周\n', min(week_data), max(week_data));
    fprintf('GNSS周内秒范围: %.3f 到 %.3f 秒\n', min(second_data), max(second_data));
    fprintf('数据采集时长: %.3f 秒\n', max(second_data) - min(second_data));
    
    % 计算数据率
    if length(second_data) > 1
        time_diff = diff(second_data);
        avg_interval = mean(time_diff);
        data_rate = 1/avg_interval;
        fprintf('平均数据间隔: %.4f 秒\n', avg_interval);
        fprintf('平均数据率: %.2f Hz\n', data_rate);
    end
    
    % 绘制结果图表
    figure;
    
    % 加速度数据
    subplot(3,2,1);
    plot(second_data, accel_x, 'r', 'DisplayName', 'X轴'); hold on;
    plot(second_data, accel_y, 'g', 'DisplayName', 'Y轴');
    plot(second_data, accel_z, 'b', 'DisplayName', 'Z轴');
    title('加速度 (m/s²) - RAWIMUB协议 (H=28)');
    xlabel('周内秒 (s)');
    ylabel('加速度 (m/s²)');
    legend;
    grid on;
    
    % 角速度数据
    subplot(3,2,2);
    plot(second_data, gyro_x, 'r', 'DisplayName', 'X轴'); hold on;
    plot(second_data, gyro_y, 'g', 'DisplayName', 'Y轴');
    plot(second_data, gyro_z, 'b', 'DisplayName', 'Z轴');
    title('角速度 (°/s) - RAWIMUB协议 (H=28)');
    xlabel('周内秒 (s)');
    ylabel('角速度 (°/s)');
    legend;
    grid on;
    
    % 数据质量统计
    subplot(3,2,3);
    pie([normal_packets, abnormal_packets], {'正常数据', '异常数据'});
    title(sprintf('IMU状态统计 (正常率: %.1f%%)', normal_packets/total_packets*100));
    
    % 加速度统计
    subplot(3,2,4);
    boxplot([accel_x, accel_y, accel_z], 'Labels', {'X轴', 'Y轴', 'Z轴'});
    title('加速度统计分布 (m/s²)');
    ylabel('加速度 (m/s²)');
    grid on;
    
    % 角速度统计
    subplot(3,2,5);
    boxplot([gyro_x, gyro_y, gyro_z], 'Labels', {'X轴', 'Y轴', 'Z轴'});
    title('角速度统计分布 (°/s)');
    ylabel('角速度 (°/s)');
    grid on;
    
    % 时间序列
    subplot(3,2,6);
    relative_time = second_data - second_data(1);
    plot(relative_time, 'k-', 'LineWidth', 1);
    title('数据包时间序列');
    xlabel('数据包序号');
    ylabel('相对时间 (s)');
    grid on;
    
    % 保存解析结果
    result_filename = [filename(1:end-4) '_rawimub_H28_parsed.mat'];
    save(result_filename, 'week_data', 'second_data', 'accel_x', 'accel_y', 'accel_z', ...
        'gyro_x', 'gyro_y', 'gyro_z', 'status_words', 'crc_values', 'IMU_DATA');
    fprintf('解析结果已保存到: %s\n', result_filename);
    
    % 显示前几个数据包的详细信息
    fprintf('\n===== 前3个正常数据包详细信息 =====\n');
    for k = 1:min(3, length(week_data))
        fprintf('数据包 %d:\n', k);
        fprintf('  GNSS周数: %d, 周内秒: %.6f s\n', week_data(k), second_data(k));
        fprintf('  状态字: 0x%08X (正常，Hex Ulong类型)\n', status_words(k));
        fprintf('  加速度: X=%.6f, Y=%.6f, Z=%.6f m/s²\n', accel_x(k), accel_y(k), accel_z(k));
        fprintf('  角速度: X=%.6f, Y=%.6f, Z=%.6f °/s\n', gyro_x(k), gyro_y(k), gyro_z(k));
        fprintf('  CRC32: 0x%08X\n', crc_values(k));
        fprintf('\n');
    end
    
    % 显示数据统计信息
    fprintf('\n===== 正常数据统计信息 =====\n');
    fprintf('加速度统计 (X轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] m/s²\n', ...
        mean(accel_x), std(accel_x), min(accel_x), max(accel_x));
    fprintf('加速度统计 (Y轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] m/s²\n', ...
        mean(accel_y), std(accel_y), min(accel_y), max(accel_y));
    fprintf('加速度统计 (Z轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] m/s²\n', ...
        mean(accel_z), std(accel_z), min(accel_z), max(accel_z));
    
    fprintf('角速度统计 (X轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] °/s\n', ...
        mean(gyro_x), std(gyro_x), min(gyro_x), max(gyro_x));
    fprintf('角速度统计 (Y轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] °/s\n', ...
        mean(gyro_y), std(gyro_y), min(gyro_y), max(gyro_y));
    fprintf('角速度统计 (Z轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] °/s\n', ...
        mean(gyro_z), std(gyro_z), min(gyro_z), max(gyro_z));
else
    fprintf('警告: 没有找到状态字为0x00的正常数据包！\n');
    fprintf('请检查数据文件或传感器状态。\n');
end

% 显示数据包结构信息
fprintf('\n===== 数据包结构信息 =====\n');
fprintf('帧头长度 H = %d 字节\n', H);
fprintf('数据包总长度 = H + 45 = %d 字节\n', H + 45);
fprintf('各字段偏移量:\n');
fprintf('  GNSS周数: 字节 %d-%d (Ulong类型)\n', H+1, H+4);
fprintf('  GNSS周内秒: 字节 %d-%d (Double类型)\n', H+5, H+12);
fprintf('  状态字: 字节 %d-%d (Hex Ulong类型)\n', H+13, H+16);
fprintf('  Z轴加速度: 字节 %d-%d (Long类型)\n', H+17, H+20);
fprintf('  Y轴加速度: 字节 %d-%d (Long类型)\n', H+21, H+24);
fprintf('  X轴加速度: 字节 %d-%d (Long类型)\n', H+25, H+28);
fprintf('  Z轴角速度: 字节 %d-%d (Long类型)\n', H+29, H+32);
fprintf('  Y轴角速度: 字节 %d-%d (Long类型)\n', H+33, H+36);
fprintf('  X轴角速度: 字节 %d-%d (Long类型)\n', H+37, H+40);
fprintf('  CRC32校验: 字节 %d-%d (Hex类型)\n', H+41, H+44);
fprintf('  结束符: 字节 %d\n', H+45);

% 显示数据类型说明
fprintf('\n===== 数据类型说明 =====\n');
fprintf('Hex Ulong: 十六进制无符号长整型 (uint32)，以十六进制格式显示\n');
fprintf('Ulong: 无符号长整型 (uint32)\n');
fprintf('Double: 双精度浮点型\n');
fprintf('Long: 有符号长整型 (int32)\n');
fprintf('Hex: 十六进制格式\n');
%% 加速度计测量模值和陀螺仪测量模值
aa=sqrt(IMU_DATA(:,5).^2+ IMU_DATA(:,6).^2+ IMU_DATA(:,4).^2);
subplot 121,plot(aa),title('陀螺仪/单位°/s')
bb=sqrt(IMU_DATA(:,1).^2+ IMU_DATA(:,2).^2+ IMU_DATA(:,3).^2);
subplot 122,plot(bb),title('加速度计/单位g')
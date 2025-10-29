% 从文件读取并解析IMU数据包（基于新协议），只处理状态字为00的正常数据
clear
filename = 'C:\Users\23764\Desktop\惯导试验\数据\20250927.txt'; % 替换为文件名

% 以二进制模式读取整个文件
fid = fopen(filename, 'rb');
allData = fread(fid, inf, 'uint8');
fclose(fid);

% 查找满足帧头条件的位置
headerPositions = [];
frameHeader = [235, 144, 32]; % 帧头: 0xEB, 0x90, 0x20

% 查找帧头位置
for i = 1:length(allData)-35 % 需要至少36字节才能构成完整数据包
    % 检查前3个字节是否符合帧头特征
    if all(allData(i:i+2) == frameHeader')
        headerPositions = [headerPositions, i];
    end
end

% 计算包间距并检查异常
if length(headerPositions) > 1
    len_eachpacket = diff(headerPositions);
    abnormal = find(len_eachpacket ~= 36);
    
    % 显示异常包信息
    if ~isempty(abnormal)
        fprintf('发现 %d 个异常长度的数据包:\n', length(abnormal));
        for i = 1:length(abnormal)
            idx = abnormal(i);
            fprintf('  包 %d: 位置=%d, 长度=%d字节\n', idx, headerPositions(idx), len_eachpacket(idx));
        end
    end
end

% 初始化存储数组
ki = 1;
time_data = [];          % 时间数据 (字节4-7)
gyro_x = [];             % X轴角速率 (字节8-11)
gyro_y = [];             % Y轴角速率 (字节12-15)
gyro_z = [];             % Z轴角速率 (字节16-19)
accel_x = [];            % X轴加速度 (字节20-23)
accel_y = [];            % Y轴加速度 (字节24-27)
accel_z = [];            % Z轴加速度 (字节28-31)
temperature = [];        % 温度 (字节32-33)
status_words = [];       % 状态字 (字节34-35)
checksums = [];          % 校验和 (字节36)

% 用于统计的变量
total_packets = length(headerPositions);
normal_packets = 0;
abnormal_packets = 0;

fprintf('开始解析IMU数据包...\n');
fprintf('找到 %d 个数据包\n', total_packets);

% 创建IMU_FUR矩阵存储结果
IMU_FUR = [];

for i = 1:length(headerPositions)
    startIdx = headerPositions(i);
    endIdx = startIdx + 35; % 36字节数据包
    
    if endIdx <= length(allData)
        dataPacket = allData(startIdx:endIdx);
        
        try
            % 验证帧头
            if dataPacket(1) == 235 && dataPacket(2) == 144 && dataPacket(3) == 32
                % 解析状态字 (字节34-35, unsigned short)
                status_val = typecast(uint8(dataPacket(34:35)), 'uint16');
                
                % 只处理状态字为0x00（正常）的数据包
                if status_val == 0
                    normal_packets = normal_packets + 1;
                    
                    % 解析时间数据 (字节4-7, unsigned int, 单位: ms)
                    time_val = typecast(uint8(dataPacket(4:7)), 'uint32');
                    time_data(ki, 1) = double(time_val);
                    
                    % 解析角速率 (字节8-19, float, 单位: °/s)
                    gyro_x_val = typecast(uint8(dataPacket(8:11)), 'single');
                    gyro_y_val = typecast(uint8(dataPacket(12:15)), 'single');
                    gyro_z_val = typecast(uint8(dataPacket(16:19)), 'single');
                    
                    gyro_x(ki, 1) = gyro_x_val;
                    gyro_y(ki, 1) = gyro_y_val;
                    gyro_z(ki, 1) = gyro_z_val;
                    
                    % 解析加速度 (字节20-31, float, 单位: m/s²)
                    accel_x_val = typecast(uint8(dataPacket(20:23)), 'single');
                    accel_y_val = typecast(uint8(dataPacket(24:27)), 'single');
                    accel_z_val = typecast(uint8(dataPacket(28:31)), 'single');
                    
                    accel_x(ki, 1) = accel_x_val;
                    accel_y(ki, 1) = accel_y_val;
                    accel_z(ki, 1) = accel_z_val;
                    
                    % 解析温度 (字节32-33, short, 单位: °C, 分辨率: 1/16)
                    temp_val = typecast(uint8(dataPacket(32:33)), 'int16');
                    temperature(ki, 1) = double(temp_val) / 16.0; % 除以16得到实际温度
                    
                    % 存储状态字
                    status_words(ki, 1) = double(status_val);
                    
                    % 解析校验和 (字节36, unsigned char)
                    checksum_val = dataPacket(36);
                    checksums(ki, 1) = checksum_val;
                    
                    % 计算校验和进行验证
                    calculated_checksum = mod(sum(double(dataPacket(3:35))), 256);
                    is_checksum_valid = (checksum_val == calculated_checksum);
                    
                    % 存储到IMU_FUR矩阵
                    % 格式: [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, time, status, checksum_valid]
                    IMU_FUR(ki, 1:9) = [gyro_x_val, gyro_y_val, gyro_z_val, ...
                                        accel_x_val, accel_y_val, accel_z_val, ...
                                        double(time_val), double(status_val), is_checksum_valid];
                    
                    ki = ki + 1;
                else
                    abnormal_packets = abnormal_packets + 1;
                    % 可选：记录异常状态字的值
                    % fprintf('数据包 %d: 状态字异常 (0x%04X)\n', i, status_val);
                end
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
    % 显示时间信息
    fprintf('时间范围: %d 到 %d ms\n', min(time_data), max(time_data));
    fprintf('数据采集时长: %.3f 秒\n', (max(time_data) - min(time_data)) / 1000);
    
    % 显示校验和验证结果
    valid_checksum_count = sum(IMU_FUR(:,9));
    fprintf('校验和正确率: %.1f%% (%d/%d)\n', ...
        valid_checksum_count/normal_packets*100, valid_checksum_count, normal_packets);
    
    % 计算数据率
    time_diff = diff(time_data) / 1000; % 转换为秒
    if ~isempty(time_diff)
        avg_interval = mean(time_diff);
        data_rate = 1/avg_interval;
        fprintf('平均数据间隔: %.4f 秒\n', avg_interval);
        fprintf('平均数据率: %.2f Hz\n', data_rate);
    end
    
    % 绘制结果图表
    figure
    
    % 角速率
    subplot(3,2,1);
    plot(time_data/1000, gyro_x, 'r', 'DisplayName', 'X轴(前)'); hold on;
    plot(time_data/1000, gyro_y, 'g', 'DisplayName', 'Y轴(上)');
    plot(time_data/1000, gyro_z, 'b', 'DisplayName', 'Z轴(右)');
    title('角速率 (°/s) - 仅正常数据');
    xlabel('时间 (秒)');
    ylabel('角速率 (°/s)');
    legend;
    grid on;
    
    % 加速度
    subplot(3,2,2);
    plot(time_data/1000, accel_x, 'r', 'DisplayName', 'X轴(前)'); hold on;
    plot(time_data/1000, accel_y, 'g', 'DisplayName', 'Y轴(上)');
    plot(time_data/1000, accel_z, 'b', 'DisplayName', 'Z轴(右)');
    title('加速度 (m/s²) - 仅正常数据');
    xlabel('时间 (秒)');
    ylabel('加速度 (m/s²)');
    legend;
    grid on;
    
    % 温度
    subplot(3,2,3);
    plot(time_data/1000, temperature, 'k-', 'LineWidth', 1);
    title('温度 (°C) - 仅正常数据');
    xlabel('时间 (秒)');
    ylabel('温度 (°C)');
    grid on;
    
    % 角速率统计
    subplot(3,2,4);
    boxplot([gyro_x, gyro_y, gyro_z], 'Labels', {'X轴(前)', 'Y轴(上)', 'Z轴(右)'});
    title('角速率统计分布 (°/s) - 仅正常数据');
    ylabel('角速率 (°/s)');
    grid on;
    
    % 加速度统计
    subplot(3,2,5);
    boxplot([accel_x, accel_y, accel_z], 'Labels', {'X轴(前)', 'Y轴(上)', 'Z轴(右)'});
    title('加速度统计分布 (m/s²) - 仅正常数据');
    ylabel('加速度 (m/s²)');
    grid on;
    
    % 数据质量统计
    subplot(3,2,6);
    pie([normal_packets, abnormal_packets], {'正常数据', '异常数据'});
    title(sprintf('数据质量统计 (正常率: %.1f%%)', normal_packets/total_packets*100));
    
    % 保存解析结果
    result_filename = [filename(1:end-4) '_normal_parsed.mat'];
    save(result_filename, 'time_data', 'gyro_x', 'gyro_y', 'gyro_z', ...
        'accel_x', 'accel_y', 'accel_z', 'temperature', 'status_words', 'IMU_FUR');
    fprintf('正常数据解析结果已保存到: %s\n', result_filename);
    
    % 显示前几个数据包的详细信息
    fprintf('\n===== 前3个正常数据包详细信息 =====\n');
    for k = 1:min(3, length(time_data))
        fprintf('数据包 %d:\n', k);
        fprintf('  时间: %d ms\n', time_data(k));
        fprintf('  角速率: X=%.3f, Y=%.3f, Z=%.3f °/s\n', gyro_x(k), gyro_y(k), gyro_z(k));
        fprintf('  加速度: X=%.3f, Y=%.3f, Z=%.3f m/s²\n', accel_x(k), accel_y(k), accel_z(k));
        fprintf('  温度: %.2f °C\n', temperature(k));
        fprintf('  状态字: 0x%04X (正常)\n', status_words(k));
        if IMU_FUR(k,9)    checksum_str = '有效'; 
        else    checksum_str = '无效';
        end
        fprintf('  校验和：%s\n', checksum_str);
        fprintf('\n');
    end
    
    % 显示数据统计信息
    fprintf('\n===== 正常数据统计信息 =====\n');
    fprintf('角速率统计 (X轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] °/s\n', ...
        mean(gyro_x), std(gyro_x), min(gyro_x), max(gyro_x));
    fprintf('角速率统计 (Y轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] °/s\n', ...
        mean(gyro_y), std(gyro_y), min(gyro_y), max(gyro_y));
    fprintf('角速率统计 (Z轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] °/s\n', ...
        mean(gyro_z), std(gyro_z), min(gyro_z), max(gyro_z));
    
    fprintf('加速度统计 (X轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] m/s²\n', ...
        mean(accel_x), std(accel_x), min(accel_x), max(accel_x));
    fprintf('加速度统计 (Y轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] m/s²\n', ...
        mean(accel_y), std(accel_y), min(accel_y), max(accel_y));
    fprintf('加速度统计 (Z轴): 均值=%.4f, 标准差=%.4f, 范围=[%.4f, %.4f] m/s²\n', ...
        mean(accel_z), std(accel_z), min(accel_z), max(accel_z));
    
    fprintf('温度统计: 均值=%.2f, 标准差=%.2f, 范围=[%.2f, %.2f] °C\n', ...
        mean(temperature), std(temperature), min(temperature), max(temperature));
else
    fprintf('警告: 没有找到状态字为0x00的正常数据包！\n');
    fprintf('请检查数据文件或传感器状态。\n');
end
%%

aa=sqrt(IMU_FUR(:,5).^2+ IMU_FUR(:,6).^2+ IMU_FUR(:,4).^2);
bb=sqrt(IMU_FUR(:,1).^2+ IMU_FUR(:,2).^2+ IMU_FUR(:,3).^2);
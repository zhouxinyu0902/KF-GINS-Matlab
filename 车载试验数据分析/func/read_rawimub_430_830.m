function IMU_DATA = read_rawimub_430_830(filename, headerPositions)
    % 读取RAWIMUB数据文件
    % 输入: filename - 文件名, headerPositions - 帧头位置数组
    % 输出: IMU_DATA - [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, week, second, status]
    
    % 根据协议表格定义参数
    H = 28; % 帧头长度（字节）
    packetSize = H + 45; % 总数据包大小 = 28字节帧头 + 45字节数据 = 73字节
    G_VAL = 9.806; % 重力加速度
    
    fprintf('读取RAWIMUB数据文件: %s\n', filename);
    
    % 读取文件数据
    try
        fid = fopen(filename, 'rb');
        allData = fread(fid, inf, 'uint8=>uint8');
        fclose(fid);
        fileSize = length(allData);
        fprintf('文件大小: %d 字节\n', fileSize);
    catch ME
        error('无法读取文件: %s', ME.message);
    end
    
    total_packets = length(headerPositions);
    fprintf('找到 %d 个有效数据包\n', total_packets);
    
    if total_packets == 0
        IMU_DATA = [];
        fprintf('未找到有效数据包\n');
        return;
    end
    
    % 预分配内存
    IMU_DATA = zeros(total_packets, 9);
    normal_packets = 0;
    abnormal_packets = 0;
    
    % 解析数据包
    fprintf('开始解析数据包...\n');
    for i = 1:total_packets
        startIdx = headerPositions(i);
        endIdx = startIdx + packetSize - 1;
        
        if endIdx > fileSize
            fprintf('数据包 %d 超出文件范围\n', i);
            continue;
        end
        
        % 提取数据包
        dataPacket = allData(startIdx:endIdx);
        
        try
            % 根据协议表格解析数据
            % 1. GNSS周数 (字节H+0到H+3)
            week_val = typecast(dataPacket(H+1:H+4), 'uint32');
            
            % 2. GNSS周内秒 (字节H+4到H+11)
            second_val = typecast(dataPacket(H+5:H+12), 'double');
            
            % 3. IMU状态字 (字节H+12到H+15)
            status_val = typecast([0;0;dataPacket(H+15:H+16)], 'uint32');
            
            % 4. Z轴加速度 (字节H+16到H+19)
            accel_z_raw = typecast(dataPacket(H+17:H+20), 'int32');
            accel_z_val = double(accel_z_raw) / (G_VAL * 655360);
            
            % 5. Y轴加速度 (字节H+20到H+23)
            accel_y_raw = typecast(dataPacket(H+21:H+24), 'int32');
            accel_y_val = double(accel_y_raw) / (G_VAL * 655360);
            
            % 6. X轴加速度 (字节H+24到H+27)
            accel_x_raw = typecast(dataPacket(H+25:H+28), 'int32');
            accel_x_val = double(accel_x_raw) / (G_VAL * 655360);
            
            % 7. Z轴角速度 (字节H+28到H+31)
            gyro_z_raw = typecast(dataPacket(H+29:H+32), 'int32');
            gyro_z_val = double(gyro_z_raw) / 160849.543863;
            
            % 8. Y轴角速度 (字节H+32到H+35)
            gyro_y_raw = typecast(dataPacket(H+33:H+36), 'int32');
            gyro_y_val = double(gyro_y_raw) / 160849.543863;
            
            % 9. X轴角速度 (字节H+36到H+39)
            gyro_x_raw = typecast(dataPacket(H+37:H+40), 'int32');
            gyro_x_val = double(gyro_x_raw) / 160849.543863;
            
            % 存储到结果矩阵 [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, week, second, status]
            IMU_DATA(i, :) = [gyro_x_val, gyro_y_val, gyro_z_val,...
                              accel_x_val, accel_y_val, accel_z_val, ...
                               second_val,double(week_val), double(status_val)];
            
            % if status_val == 0
            %     normal_packets = normal_packets + 1;
            % else
            %     abnormal_packets = abnormal_packets + 1;
            %     if mod(abnormal_packets, 100) == 0
            %         fprintf('数据包 %d: 状态字异常 (0x%08X)\n', i, status_val);
            %     end
            % end
            
            % 显示进度
            % if mod(i, 10000) == 0
            %     fprintf('已解析 %d/%d 数据包\n', i, total_packets);
            % end
            
        catch ME
            fprintf('解析数据包 %d 时出错: %s\n', i, ME.message);
            continue;
        end
    end
    
    % 移除未解析的数据包
    valid_rows = any(IMU_DATA ~= 0, 2);
    IMU_DATA = IMU_DATA(valid_rows, :);
    
    % % 显示统计结果
    % fprintf('\n===== 解析结果统计 =====\n');
    fprintf('总数据包数量: %d 个\n', total_packets);
    % fprintf('正常数据包 (状态字=0x00): %d 个\n', normal_packets);
    % fprintf('异常数据包 (状态字≠0x00): %d 个\n', abnormal_packets);
    % fprintf('正常数据比例: %.2f%%\n', normal_packets/total_packets*100);
    % fprintf('数据解析完成，返回 %d×9 矩阵\n', size(IMU_DATA, 1));
end
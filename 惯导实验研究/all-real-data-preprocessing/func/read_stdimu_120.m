function IMU_FUR = read_stdimu_120(filename, headerPositions)

    % 使用内存映射进行极速解析
    
    % 创建内存映射
    m = memmapfile(filename, 'Format', 'uint8');
    data = m.Data;
    
    num_packets = length(headerPositions);
    IMU_FUR = zeros(num_packets, 9);
    valid_count = 0;
    
    % 预计算有效范围
    valid_indices = headerPositions(headerPositions + 35 <= length(data));
    
    for i = 1:length(valid_indices)
        idx = valid_indices(i);
        
        % 直接访问映射内存（最快）
        if data(idx) == 235 && data(idx+1) == 144 && data(idx+2) == 32
            status_bytes = [data(idx+33), data(idx+34)];
            status_val = typecast(uint8(status_bytes), 'uint16');
            
            if status_val == 0
                valid_count = valid_count + 1;
                
                % 解析数据
                IMU_FUR(valid_count, 1) = typecast(uint8(data(idx+7:idx+10)), 'single');
                IMU_FUR(valid_count, 2) = typecast(uint8(data(idx+11:idx+14)), 'single');
                IMU_FUR(valid_count, 3) = typecast(uint8(data(idx+15:idx+18)), 'single');
                IMU_FUR(valid_count, 4) = typecast(uint8(data(idx+19:idx+22)), 'single');
                IMU_FUR(valid_count, 5) = typecast(uint8(data(idx+23:idx+26)), 'single');
                IMU_FUR(valid_count, 6) = typecast(uint8(data(idx+27:idx+30)), 'single');
                IMU_FUR(valid_count, 7) = double(typecast(uint8(data(idx+3:idx+6)), 'uint32'));
                IMU_FUR(valid_count, 8) = double(status_val);
                
                % 校验和
                checksum_val = data(idx+35);
                packet_sum = sum(double(data(idx+2:idx+34)));
                IMU_FUR(valid_count, 9) = (checksum_val == mod(packet_sum, 256));
            end
        end
    end
    
    IMU_FUR = IMU_FUR(1:valid_count, :);
end


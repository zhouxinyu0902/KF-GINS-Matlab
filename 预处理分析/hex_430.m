% 从文件读取并解析多个RawB数据包
clear
filename = '111800_raw.txt'; % 替换为您的文件名

% 以二进制模式读取整个文件
fid = fopen(filename, 'rb');
allData = fread(fid, inf, 'uint8');
fclose(fid);

% 计算数据包数量
packetSize = 149;
numPackets = floor(length(allData) / packetSize);
data=find(allData==235);
eachpacketlength=diff(data);
indexnormal=find(eachpacketlength==148);

datanormal=data(indexnormal);
for i=1:length(datanormal)
    d(:,i)=allData(datanormal(i):datanormal(i)+148);
end
fprintf('文件中共有 %d 个数据包\n', numPackets);

% 解析每个数据包
for i = 1:numPackets
    fprintf('\n===== 解析第 %d 个数据包 =====\n', i);
    
    % 提取当前数据包
    startIdx = (i-1)*packetSize + 1;
    endIdx = i*packetSize;
    dataPacket = allData(startIdx:endIdx);
    
    try
        % 这里可以调用上面的解析代码，或者将上面的代码封装成函数
        % 为简洁起见，这里只显示关键数据
        if dataPacket(1) == 235 && dataPacket(2) == 144
            % 解析角度
            roll = double(typecast(dataPacket(27:28), 'int16')) * (180.0 / 32767);
            pitch = double(typecast(dataPacket(31:32), 'int16')) * (90.0 / 32767);
            
            fprintf('数据包 %d: Roll=%.2f°, Pitch=%.2f°\n', i, roll, pitch);
        else
            fprintf('数据包 %d: 帧头错误\n', i);
        end
    catch ME
        fprintf('解析数据包 %d 时出错: %s\n', i, ME.message);
    end
end
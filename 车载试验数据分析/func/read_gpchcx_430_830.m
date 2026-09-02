function GPCHCX_430 = read_gpchcx_430_830(filename1)
% 一次性读取整个文件
fid = fopen(filename1, 'rt');
all_text = fread(fid, '*char')';
fclose(fid);
fprintf('    (2)读取GPCHCX数据,')
% 分割成行
lines = strsplit(all_text, '\n');

% 预分配内存（根据文件大小估计）
estimated_lines = length(lines);
GPCHCX_430 = zeros(46, estimated_lines);
valid_mask = false(1, estimated_lines);
valid_count = 0;

% 批量处理所有行
for i = 1:length(lines)
    line = lines{i};
    
    % 检查是否为GPCHCX数据行
    if ~isempty(line) && contains(line, '$GPCHCX')
        try
            % 尝试解析数据
            data = sscanf(line, '$GPCHCX,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%X,%f,%f,%f,%f,%f,%f,%f,%f,%f,%c,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d*%X');
            
            if length(data) == 46
                valid_count = valid_count + 1;
                GPCHCX_430(:, valid_count) = data;
                valid_mask(valid_count) = true;
            else
                fprintf('警告: 第%d行数据格式不匹配（期望46列，得到%d列），跳过\n', i, length(data));
            end
        catch ME
            fprintf('错误: 第%d行解析失败: %s\n', i, ME.message);
        end
    end
end

% 提取有效数据
GPCHCX_430 = GPCHCX_430(:, 1:valid_count);

% 显示结果信息
fprintf('成功解析 %d 条有效记录（总共 %d 行）\n', valid_count, length(lines)-1);
end
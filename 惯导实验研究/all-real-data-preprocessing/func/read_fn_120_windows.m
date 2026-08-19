function FN_120_windows = read_fn_120_windows(filename)
% 一次性读取整个文件
fid = fopen(filename, 'rt');
all_text = fread(fid, '*char')';
fclose(fid);

% 分割成行
lines = strsplit(all_text, '\n');

% 跳过前6行
if length(lines) > 6
    lines = lines(6:end);
else
    error('文件行数不足，无法跳过前6行');
end

% 预分配内存（根据文件大小估计）
estimated_lines = length(lines);
FN_120_windows = zeros(33, estimated_lines); % 33列数据
valid_mask = false(1, estimated_lines);
valid_count = 0;

% 批量处理所有行
for i = 1:length(lines)
    line = lines{i};
    
    % 检查是否为有效数据行（包含数字和空格）
    if ~isempty(line) && ~all(isspace(line)) && any(isstrprop(line, 'digit'))
        try
            % 尝试解析数据（33个浮点数）
            data = sscanf(line, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
            
            if length(data) == 33
                valid_count = valid_count + 1;
                FN_120_windows(:, valid_count) = data; % 填充所有33列
                valid_mask(valid_count) = true;
            else
                fprintf('警告: 第%d行数据格式不匹配（期望33列，得到%d列），跳过\n', i + 6, length(data));
            end
        catch ME
            fprintf('错误: 第%d行解析失败: %s\n', i + 6, ME.message);
        end
    end
end

% 提取有效数据
FN_120_windows = FN_120_windows(:, 1:valid_count);

% 显示结果信息
fprintf('成功解析 %d 条有效记录（总共 %d 行）\n', valid_count, length(lines)-1);

end
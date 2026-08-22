function GPGGA_430=read_gpgga_430_830(filename2)
% 一次性读取整个文件
fid = fopen(filename2, 'rt');
all_text = fread(fid, '*char')';
fclose(fid);
% 分割成行
lines = strsplit(all_text, '\n');

% 预分配内存
GPGGA_data = zeros(18, length(lines));
valid_mask = false(1, length(lines));

% 并行处理或批量处理
parfor i = 1:length(lines)
    line = lines{i};
    if ~isempty(line) && contains(line, '$GPGGA')
        data = sscanf(line, '$GPGGA,%2d%2d%f,%2d%f,%c,%3d%f,%c,%d,%d,%f,%f,%c,%f,%c,%f,0000*%X');
        if length(data) == 18
            GPGGA_data(:, i) = data;
            valid_mask(i) = true;
        end
    end
end

% 提取有效数据
GPGGA_430 = GPGGA_data(:, valid_mask);
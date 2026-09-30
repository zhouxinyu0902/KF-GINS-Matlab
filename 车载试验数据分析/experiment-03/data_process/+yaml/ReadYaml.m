function data = ReadYaml(file_path)
%READYAML 读取 experiment-03 初始状态使用的简单 YAML 键值文件。
% 支持数值标量和一维数值数组，接口与现有 yaml.ReadYaml 调用兼容。

    if ~isfile(file_path)
        error('YAML 文件不存在：%s', file_path);
    end

    lines = splitlines(string(fileread(file_path)));
    data = struct();
    for line_index = 1:numel(lines)
        line = strtrim(extractBefore(lines(line_index) + "#", "#"));
        if strlength(line) == 0
            continue;
        end
        separator = strfind(line, ":");
        if isempty(separator)
            continue;
        end

        key = strtrim(extractBefore(line, separator(1)));
        raw_value = strtrim(extractAfter(line, separator(1)));
        if strlength(key) == 0 || strlength(raw_value) == 0
            continue;
        end

        field_name = matlab.lang.makeValidName(char(key));
        data.(field_name) = parse_value(raw_value, field_name, file_path);
    end
end

function value = parse_value(raw_value, field_name, file_path)
%PARSE_VALUE 解析数值标量或方括号数值数组。

    if startsWith(raw_value, "[") && endsWith(raw_value, "]")
        content = extractBetween(raw_value, 2, strlength(raw_value) - 1);
        tokens = split(replace(content, ",", " "));
        tokens = tokens(strlength(strtrim(tokens)) > 0);
        numbers = str2double(tokens);
        if isempty(numbers) || any(~isfinite(numbers))
            error('YAML 数组字段 %s 不是有效数值：%s', ...
                field_name, file_path);
        end
        value = num2cell(numbers(:).');
        return;
    end

    number = str2double(raw_value);
    if isfinite(number)
        value = number;
    else
        value = char(raw_value);
    end
end

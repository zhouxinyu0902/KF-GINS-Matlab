clear;
clc;
close all;

%% static-1 / static-2 FN-120 AUXA 状态字分析
% 本文件是可直接运行的脚本，不是函数。
%
% AUXA 的第 32 个数值字段是流程控制状态字（read_auax_120 的第 32 行）。
% static-1：依据对应协议图，只把低 16 bit 当作有效状态字；高 16 bit 不解释。
% static-2：依据“流程控制状态字”表，解释完整 32 bit，包括 bit16~bit31 故障字。
%
% 注意：static-1 协议图只给出了状态字所在字节，并指向未提供的“表 10”。
% 因此脚本保留 static-1 的原始低 16 bit 统计，并用 static-2 表中相同的低
% 16 bit 布局作辅助解释。原始 HEX 值和状态跳变不依赖该辅助解释。

script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(fileparts(script_dir));
func_dir = fullfile(fileparts(script_dir), 'func');
data_root = fullfile(repo_dir, 'data', 'experiment-data');
addpath(func_dir);

dataset_names = {'static-1', 'static-2'};
protocol_width_bits = [16, 32];
status_row = 32;

% 图 1 给出的 bit16~bit26 故障定义。bit27~bit31 按保留位统计。
fault_bit_numbers = (16:31).';
fault_names = {
    'X陀螺异常'
    'Y陀螺异常'
    'Z陀螺异常'
    'X加速度计异常'
    'Y加速度计异常'
    'Z加速度计异常'
    '温度异常'
    'GPS通讯异常'
    'PPS异常'
    'DVL通讯异常（仅DVL组合模式判断）'
    '双天线航向参考安装误差太大'
    '保留位 bit27'
    '保留位 bit28'
    '保留位 bit29'
    '保留位 bit30'
    '保留位 bit31'
    };

fprintf('\n============================================================\n');
fprintf('FN-120 AUXA 状态字分析：static-1 与 static-2\n');
fprintf('============================================================\n');

for dataset_index = 1:numel(dataset_names)
    dataset_name = dataset_names{dataset_index};
    protocol_bits = protocol_width_bits(dataset_index);
    data_file = fullfile(data_root, dataset_name, 'Disk1_000.dat');
    output_dir = fullfile(data_root, dataset_name, 'output', 'figures-tables');

    if exist(data_file, 'file') ~= 2
        error('找不到 AUXA 文件：%s', data_file);
    end
    if exist(output_dir, 'dir') ~= 7
        mkdir(output_dir);
    end

    fprintf('\n------------------------------------------------------------\n');
    fprintf('%s\n', dataset_name);
    fprintf('文件：%s\n', data_file);

    auxa = read_auax_120(data_file);
    if size(auxa, 1) < status_row || isempty(auxa)
        error('%s 未读到有效的 34 列 AUXA 数据。', dataset_name);
    end

    record_count = size(auxa, 2);
    record_index = (1:record_count).';
    machine_time = auxa(1, :).';
    gps_sow = auxa(2, :).';
    raw_status = uint32(auxa(status_row, :).');
    low16_status = bitand(raw_status, uint32(hex2dec('FFFF')));

    if protocol_bits == 16
        effective_status = low16_status;
        fault_word = zeros(record_count, 1, 'uint32');
        protocol_note = '图2：仅分析低16 bit；低16位含义借用图1布局作辅助解释';
    else
        effective_status = raw_status;
        fault_word = bitshift(raw_status, -16);
        protocol_note = '图1：分析完整32 bit（低16位流程状态 + 高16位故障状态）';
    end

    bit_status = bitand(low16_status, uint32(15));
    work_state = bitand(bitshift(low16_status, -4), uint32(15));
    horizontal_mode = bitand(bitshift(low16_status, -8), uint32(15));
    vertical_mode = bitand(bitshift(low16_status, -12), uint32(3));
    binding_mode = bitand(bitshift(low16_status, -14), uint32(3));

    time_step = diff(machine_time);
    positive_time_step = time_step(isfinite(time_step) & time_step > 0 & time_step < 1);
    if isempty(positive_time_step)
        nominal_dt = NaN;
    else
        nominal_dt = median(positive_time_step);
    end

    % 上机时间不递增表示设备重新开机。每个开机段独立编号。
    boot_segment = cumsum([1; time_step <= 0]);
    segment_start_index = [1; find(time_step <= 0) + 1];
    segment_end_index = [segment_start_index(2:end) - 1; record_count];

    segment_table = table(...
        (1:numel(segment_start_index)).', ...
        segment_start_index, segment_end_index, ...
        machine_time(segment_start_index), machine_time(segment_end_index), ...
        segment_end_index - segment_start_index + 1, ...
        'VariableNames', {'segment_id', 'first_record', 'last_record', ...
        'start_machine_time_s', 'end_machine_time_s', 'record_count'});
    writetable(segment_table, fullfile(output_dir, 'auxa_status_boot_segments.csv'), ...
        'Encoding', 'UTF-8');

    %% 唯一状态字统计
    [unique_status, ~, unique_group] = unique(effective_status, 'sorted');
    status_count = accumarray(unique_group, 1);
    unique_count = numel(unique_status);

    raw_hex = cell(unique_count, 1);
    low16_hex = cell(unique_count, 1);
    bit_code = zeros(unique_count, 1);
    bit_meaning = cell(unique_count, 1);
    work_code = zeros(unique_count, 1);
    work_meaning = cell(unique_count, 1);
    horizontal_code = zeros(unique_count, 1);
    horizontal_meaning = cell(unique_count, 1);
    vertical_code = zeros(unique_count, 1);
    vertical_meaning = cell(unique_count, 1);
    binding_code = zeros(unique_count, 1);
    binding_meaning = cell(unique_count, 1);
    fault_hex = cell(unique_count, 1);
    active_faults = cell(unique_count, 1);
    complete_meaning = cell(unique_count, 1);

    for state_index = 1:unique_count
        state_value = unique_status(state_index);
        state_low16 = bitand(state_value, uint32(hex2dec('FFFF')));
        state_fault_word = bitshift(state_value, -16);

        bit_code(state_index) = double(bitand(state_low16, uint32(15)));
        work_code(state_index) = double(bitand(bitshift(state_low16, -4), uint32(15)));
        horizontal_code(state_index) = double(bitand(bitshift(state_low16, -8), uint32(15)));
        vertical_code(state_index) = double(bitand(bitshift(state_low16, -12), uint32(3)));
        binding_code(state_index) = double(bitand(bitshift(state_low16, -14), uint32(3)));

        switch bit_code(state_index)
            case 0
                bit_meaning{state_index} = '正常';
            case 15
                bit_meaning{state_index} = '失败';
            otherwise
                bit_meaning{state_index} = sprintf('未定义(0x%X)', bit_code(state_index));
        end

        switch work_code(state_index)
            case 10
                work_meaning{state_index} = '初始化';
            case 9
                work_meaning{state_index} = '自检';
            case 8
                work_meaning{state_index} = '等待装订';
            case 6
                work_meaning{state_index} = '装订完成';
            case 4
                work_meaning{state_index} = '粗对准';
            case 2
                work_meaning{state_index} = '精对准';
            case 1
                work_meaning{state_index} = '对准好';
            case 0
                work_meaning{state_index} = '导航';
            otherwise
                work_meaning{state_index} = sprintf('未定义(0x%X)', work_code(state_index));
        end

        switch horizontal_code(state_index)
            case 0
                horizontal_meaning{state_index} = '纯惯性导航';
            case 1
                horizontal_meaning{state_index} = '位置组合';
            case 2
                horizontal_meaning{state_index} = '速度组合';
            case 3
                horizontal_meaning{state_index} = '速度+位置组合';
            case 4
                horizontal_meaning{state_index} = '速度+位置+航向组合';
            case 8
                horizontal_meaning{state_index} = '惯导静止';
            case 6
                horizontal_meaning{state_index} = '速度+位置+航向+自动静态组合';
            case 9
                horizontal_meaning{state_index} = '罗经';
            case 11
                horizontal_meaning{state_index} = 'DVL组合';
            otherwise
                horizontal_meaning{state_index} = sprintf('未定义(0x%X)', horizontal_code(state_index));
        end

        switch vertical_code(state_index)
            case 0
                vertical_meaning{state_index} = '纯惯性导航';
            case 1
                vertical_meaning{state_index} = '卫星（无效时不跟踪）';
            case 2
                vertical_meaning{state_index} = '气压';
            case 3
                vertical_meaning{state_index} = '自动保持在装订';
        end

        switch binding_code(state_index)
            case 0
                binding_meaning{state_index} = '等待10 s';
            case 1
                binding_meaning{state_index} = '等待GNSS位置';
            case 2
                binding_meaning{state_index} = '手册未定义(值2)';
            case 3
                binding_meaning{state_index} = '等待GNSS航向';
        end

        raw_hex{state_index} = sprintf('0x%08X', state_value);
        low16_hex{state_index} = sprintf('0x%04X', state_low16);
        fault_hex{state_index} = sprintf('0x%04X', state_fault_word);

        fault_list = {};
        if protocol_bits == 32
            for fault_index = 1:numel(fault_bit_numbers)
                if bitget(state_value, fault_bit_numbers(fault_index) + 1) ~= 0
                    fault_list{end + 1} = fault_names{fault_index}; %#ok<SAGROW>
                end
            end
            if isempty(fault_list)
                active_faults{state_index} = '无';
            else
                active_faults{state_index} = strjoin(fault_list, '；');
            end
        else
            active_faults{state_index} = '未提供/不解释高16位';
        end

        complete_meaning{state_index} = strjoin({... 
            ['BIT状态=' bit_meaning{state_index}], ...
            ['工作状态=' work_meaning{state_index}], ...
            ['水平模式=' horizontal_meaning{state_index}], ...
            ['垂直模式=' vertical_meaning{state_index}], ...
            ['装订方式=' binding_meaning{state_index}], ...
            ['故障=' active_faults{state_index}]}, '；');
    end

    percentage = 100 * status_count / record_count;
    estimated_duration_s = status_count * nominal_dt;
    status_summary = table(raw_hex, low16_hex, status_count, percentage, ...
        estimated_duration_s, bit_code, bit_meaning, work_code, work_meaning, ...
        horizontal_code, horizontal_meaning, vertical_code, vertical_meaning, ...
        binding_code, binding_meaning, fault_hex, active_faults, complete_meaning, ...
        'VariableNames', {'raw_status_hex', 'low16_hex', 'record_count', ...
        'percentage', 'estimated_duration_s', 'bit_code', 'bit_meaning', ...
        'work_code', 'work_meaning', 'horizontal_code', 'horizontal_meaning', ...
        'vertical_code', 'vertical_meaning', 'binding_code', 'binding_meaning', ...
        'fault_word_hex', 'active_faults', 'complete_meaning'});
    writetable(status_summary, fullfile(output_dir, 'auxa_status_summary.csv'), ...
        'Encoding', 'UTF-8');

    %% 状态跳变（同时把重新开机作为一个事件）
    status_changed = diff(double(effective_status)) ~= 0;
    boot_changed = time_step <= 0;
    transition_index = [1; find(status_changed | boot_changed) + 1];
    transition_count = numel(transition_index);

    transition_raw_hex = cell(transition_count, 1);
    transition_low16_hex = cell(transition_count, 1);
    transition_fault_hex = cell(transition_count, 1);
    transition_event = repmat({'状态跳变'}, transition_count, 1);
    transition_description = cell(transition_count, 1);
    transition_event{1} = '文件起点';

    for transition_number = 1:transition_count
        sample_index = transition_index(transition_number);
        state_value = effective_status(sample_index);
        state_low16 = low16_status(sample_index);
        transition_raw_hex{transition_number} = sprintf('0x%08X', state_value);
        transition_low16_hex{transition_number} = sprintf('0x%04X', state_low16);
        transition_fault_hex{transition_number} = sprintf('0x%04X', fault_word(sample_index));
        summary_row = find(unique_status == state_value, 1, 'first');
        transition_description{transition_number} = complete_meaning{summary_row};
        if sample_index > 1 && boot_changed(sample_index - 1)
            transition_event{transition_number} = '重新开机/时间回跳';
        end
    end

    transition_table = table(transition_index, boot_segment(transition_index), ...
        machine_time(transition_index), gps_sow(transition_index), ...
        transition_raw_hex, transition_low16_hex, ...
        double(bit_status(transition_index)), double(work_state(transition_index)), ...
        double(horizontal_mode(transition_index)), double(vertical_mode(transition_index)), ...
        double(binding_mode(transition_index)), transition_fault_hex, ...
        transition_event, transition_description, ...
        'VariableNames', {'record_index', 'segment_id', 'machine_time_s', ...
        'gps_sow_s', 'raw_status_hex', 'low16_hex', 'bit_code', 'work_code', ...
        'horizontal_code', 'vertical_code', 'binding_code', 'fault_word_hex', ...
        'event', 'complete_meaning'});
    writetable(transition_table, fullfile(output_dir, 'auxa_status_transitions.csv'), ...
        'Encoding', 'UTF-8');

    %% 故障位统计
    fault_record_count = zeros(numel(fault_bit_numbers), 1);
    fault_percentage = zeros(numel(fault_bit_numbers), 1);
    first_record = nan(numel(fault_bit_numbers), 1);
    last_record = nan(numel(fault_bit_numbers), 1);
    first_machine_time_s = nan(numel(fault_bit_numbers), 1);
    last_machine_time_s = nan(numel(fault_bit_numbers), 1);

    if protocol_bits == 32
        for fault_index = 1:numel(fault_bit_numbers)
            active_index = find(bitget(raw_status, fault_bit_numbers(fault_index) + 1) ~= 0);
            fault_record_count(fault_index) = numel(active_index);
            fault_percentage(fault_index) = 100 * numel(active_index) / record_count;
            if ~isempty(active_index)
                first_record(fault_index) = active_index(1);
                last_record(fault_index) = active_index(end);
                first_machine_time_s(fault_index) = machine_time(active_index(1));
                last_machine_time_s(fault_index) = machine_time(active_index(end));
            end
        end
    end

    fault_table = table(fault_bit_numbers, fault_names, fault_record_count, ...
        fault_percentage, first_record, last_record, first_machine_time_s, ...
        last_machine_time_s, ...
        'VariableNames', {'bit_number', 'meaning', 'record_count', 'percentage', ...
        'first_record', 'last_record', 'first_machine_time_s', 'last_machine_time_s'});
    writetable(fault_table, fullfile(output_dir, 'auxa_fault_summary.csv'), ...
        'Encoding', 'UTF-8');

    %% 只用跳变点绘制状态时间线，避免对几十万点重复作图
    plot_index = unique([transition_index; record_count]);
    if isfinite(nominal_dt)
        elapsed_time_s = (double(plot_index) - 1) * nominal_dt;
    else
        elapsed_time_s = double(plot_index) - 1;
    end
    known_fault_count = zeros(numel(plot_index), 1);
    if protocol_bits == 32
        for fault_index = 1:11
            known_fault_count = known_fault_count + ...
                double(bitget(raw_status(plot_index), fault_bit_numbers(fault_index) + 1));
        end
    end

    fig = figure('Visible', 'off', 'Color', 'w', ...
        'Name', [dataset_name ' AUXA status timeline'], ...
        'Position', [100, 100, 1150, 760]);
    layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(layout);
    stairs(elapsed_time_s, double(work_state(plot_index)), 'LineWidth', 1.3);
    grid on;
    ylabel('工作状态代码');
    title(sprintf('%s AUXA 状态字（%s）', dataset_name, protocol_note), ...
        'Interpreter', 'none');

    nexttile(layout);
    stairs(elapsed_time_s, double(horizontal_mode(plot_index)), 'LineWidth', 1.2);
    hold on;
    stairs(elapsed_time_s, double(vertical_mode(plot_index)), 'LineWidth', 1.2);
    stairs(elapsed_time_s, double(binding_mode(plot_index)), 'LineWidth', 1.2);
    grid on;
    ylabel('模式代码');
    legend({'水平组合', '垂直组合', '装订方式'}, 'Location', 'best');

    nexttile(layout);
    stairs(elapsed_time_s, known_fault_count, 'LineWidth', 1.3);
    grid on;
    ylabel('有效故障位数');
    xlabel('按记录数和中位采样周期换算的累计时间 (s)');

    exportgraphics(fig, fullfile(output_dir, 'auxa_status_timeline.png'), ...
        'Resolution', 180);
    savefig(fig, fullfile(output_dir, 'auxa_status_timeline.fig'));
    close(fig);

    fprintf('协议口径：%s\n', protocol_note);
    fprintf('有效记录：%d；开机段：%d；中位采样周期：%.6f s\n', ...
        record_count, height(segment_table), nominal_dt);
    fprintf('状态字取值：%d 种；状态跳变/重启事件：%d 个\n', ...
        height(status_summary), height(transition_table));
    disp(status_summary(:, {'raw_status_hex', 'record_count', 'percentage', ...
        'work_meaning', 'horizontal_meaning', 'vertical_meaning', ...
        'binding_meaning', 'active_faults'}));

    if protocol_bits == 32
        active_fault_rows = fault_table.record_count > 0;
        if any(active_fault_rows)
            fprintf('实际置位的故障位：\n');
            disp(fault_table(active_fault_rows, ...
                {'bit_number', 'meaning', 'record_count', 'percentage'}));
        else
            fprintf('没有检测到已置位的故障位。\n');
        end
    else
        fprintf('static-1 按图2只解释低16位，未把高16位作为故障字。\n');
    end
    fprintf('结果目录：%s\n', output_dir);
end

fprintf('\n分析完成。\n');

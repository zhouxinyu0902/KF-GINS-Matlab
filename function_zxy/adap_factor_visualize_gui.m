function adap_factor_visualize_gui_transposed(z, z_back)

    % 1. 创建主 Figure 窗口
    f = uifigure('Name', '自适应因子可视化 (转置显示)', ...
                 'Position', [100 100 1400 800], ... % 调整宽度以适应转置后的行数
                 'Tag', 'AdapFactorVisualizeApp');

    % 2. 准备数据和列名
    
    % --- 子表行名 (转置后作为行名) ---
    row_names_sub = {'新息残差', '量测更新后的残差', 'd²', 'χ²阈值', 'α', '异常'};
    
    % --- 主表行名 (转置后作为行名) ---
    row_names_main = {'前向: 新息残差', '前向: 量测残差', '后向: 新息残差', '后向: 量测残差'};

    % 3. 创建并展示 "前向滤波" 表格
    
    % ****** 核心修改：使用 z' 进行转置 ******
    z_transposed = z'; 
    num_cols = size(z_transposed, 2); % 原始数据的行数 (时间步长)
    col_names_t1 = arrayfun(@(i) sprintf('Time Step %d', i), 1:num_cols, 'UniformOutput', false);

    title1 = uilabel(f, 'Text', '前向滤波 (转置)', 'Position', [50 740 300 30], 'FontSize', 14, 'FontWeight', 'bold');
    
    table1_pos = [20 500 1360 220]; % 调整位置和宽度
    
    uit1 = uitable(f, ...
        'Data', z_transposed, ... % 使用转置数据
        'ColumnName', col_names_t1, ... % 列名为时间步长
        'RowName', row_names_sub, ... % 行名为原始的列名
        'Position', table1_pos);


    % 4. 创建并展示 "后向滤波" 表格
    
    % ****** 核心修改：使用 z_back' 进行转置 ******
    z_back_transposed = z_back';
    col_names_t2 = col_names_t1; % 时间步长列名相同

    title2 = uilabel(f, 'Text', '后向滤波 (转置)', 'Position', [50 460 300 30], 'FontSize', 14, 'FontWeight', 'bold');
    
    table2_pos = [20 220 1360 220]; 
    
    uit2 = uitable(f, ...
        'Data', z_back_transposed, ... % 使用转置数据
        'ColumnName', col_names_t2, ...
        'RowName', row_names_sub, ...
        'Position', table2_pos);

    
    % 5. 处理并展示 "前向，后向滤波" 比较表
    
    % 准备比较数据
    zz = zeros(length(z)+1, 6);
    zz_back = zeros(length(z)+1, 6);
    
    zz(2:end, :) = z;
    zz_back(1:end-1, :) = z_back;
    
    % 构建最终比较矩阵 (前向1, 前向2, 后向1, 后向2)
    comparison_data = [zz(:, 1:2), zz_back(:, 1:2)];

    % ****** 核心修改：使用 comparison_data' 进行转置 ******
    comparison_transposed = comparison_data';
    
    % 重新计算比较表的时间步长列名 (现在是 length(z) + 1 步)
    num_comp_cols = size(comparison_transposed, 2); 
    col_names_comp = arrayfun(@(i) sprintf('Comp Step %d', i), 1:num_comp_cols, 'UniformOutput', false);


    title3 = uilabel(f, 'Text', '前向/后向滤波关键残差对比 (转置)', 'Position', [50 180 400 30], 'FontSize', 14, 'FontWeight', 'bold');
    
    table3_pos = [20 20 1360 150]; % 调整位置和宽度
    
    uit3 = uitable(f, ...
        'Data', comparison_transposed, ... % 使用转置数据
        'ColumnName', col_names_comp, ... % 列名为时间步长
        'RowName', row_names_main, ... % 行名为原始的列名
        'Position', table3_pos);

end
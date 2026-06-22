%% =========================================================================
% 导航误差统计与报告批量导出 (1~8 组)
% =========================================================================
clear; clc; close all;

% 基础路径定义
base_input_dir = 'F:/2_Data/惯导试验/实验数据/All_data';
base_output_dir = 'D:\Github\KF-GINS-Matlab\new_惯导试验\output';

target_rounds = 1:8; % 处理范围

for id = target_rounds
    fprintf('\n🚀 正在处理第 %d 组统计报告...\n', id);
    
    % 1. 动态拼接输入和输出路径
    current_out_dir = fullfile(base_output_dir, sprintf('output%d', id));
    current_input_dir = fullfile(base_input_dir, sprintf('input%d', id));
    
    % 2. 定义文件路径 (使用 fullfile 自动处理斜杠)
    path1 = fullfile(current_out_dir, 'Origin-rad.nav');
    path2 = fullfile(current_out_dir, 'RTS-DoubleSmooth-rad.nav');
    path3 = fullfile(current_out_dir, 'RTS-SingleSmooth-rad.nav');
    path4 = fullfile(current_out_dir, 'Linear-SingleSmooth-rad.nav');
    truth1 = fullfile(current_input_dir, 'truth.nav');
    
    % 3. 检查文件是否存在
    if ~exist(truth1, 'file') || ~exist(path1, 'file')
        warning('第 %d 组数据缺失(truth.nav 或 Origin-rad.nav)，跳过...', id);
        continue;
    end
    
    % 4. 执行计算
    % 注意：你需要确保你的 calc_radial_error_gjb 函数返回的 fig 变量名称一致
    if id == 6
    [fig, finalExcelData] = calc_radial_error_gjb(truth1, path1, path2, path3, path4);
    else
    [fig, finalExcelData] = calc_radial_error_gjb(truth1, path1, path2, path3);
    end
    % 5. 导出 Excel
    outputExcelName = fullfile(current_out_dir, sprintf('导航系统径向误差统计报告-rad-%d.xlsx', id));
    writecell(finalExcelData, outputExcelName);
    
    % 6. 导出高清图片
    outputImageName = fullfile(current_out_dir, sprintf('补偿前后误差对比-rad-%d.png', id));
    exportgraphics(fig, outputImageName, 'Resolution', 600);
    
    fprintf('✅ 第 %d 组报告导出完成。\n', id);
end

fprintf('\n🎉 所有 8 组统计报告已生成完毕！\n');
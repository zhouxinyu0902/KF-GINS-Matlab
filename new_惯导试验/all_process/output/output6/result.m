clear all
base_input_dir = 'F:/2_Data/惯导试验/实验数据/All_data/input6';
base_output_dir = 'D:\Github\KF-GINS-Matlab\new_惯导试验\output\output6';
path1 = fullfile(base_output_dir, 'Origin-rad.nav');
path2 = fullfile(base_output_dir, 'RTS-DoubleSmooth-rad.nav');
path3 = fullfile(base_output_dir, 'RTS-SingleSmooth-rad.nav');
% path4 = fullfile(base_output_dir, 'Linear-DoubleSmooth-rad.nav');
path5 = fullfile(base_output_dir, 'Linear-SingleSmooth-rad.nav');
truth1 = fullfile(base_input_dir, 'truth.nav');

[fig, finalExcelData] = calc_radial_error_gjb(truth1, path1, path2, path3, path5);
id=6;
% 5. 导出 Excel
outputExcelName = fullfile(base_output_dir, sprintf('导航系统径向误差统计报告-RTS-LINEAR-%d.xlsx', id));
writecell(finalExcelData, outputExcelName);

% 6. 导出高清图片
outputImageName = fullfile(base_output_dir, sprintf('补偿前后误差对比-RTS-LINEAR-%d.png', id));
exportgraphics(fig, outputImageName, 'Resolution', 600);
clear;
close all;
clc;

%% 实测输入数据与信标场景评价
% dataget.m 只生成高度和距离数据；本脚本集中完成数据检查与绘图。

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
setup_inertial_experiment();
cfg = ProcessConfig_exper();
input_dir = cfg.inputfolder;
figure_dir = cfg.figurefolder;
if ~isfolder(figure_dir)
    mkdir(figure_dir);
end

pva_830 = readmatrix(fullfile(input_dir, 'pva_830.txt'));
std_430 = readmatrix(fullfile(input_dir, 'std_430.txt'));
std_830 = readmatrix(fullfile(input_dir, 'std_830.txt'));

%% 定位标准差对比
error_figure = myfigurestartup(4, 5, 'paper');
error_names = {'纬度误差（m）', '经度误差（m）', '高度误差（m）'};
for component_index = 1:3
    subplot(3, 1, component_index);
    plot(std_830(:, 1), std_830(:, component_index + 1), 'r.');
    hold on;
    plot(std_430(:, 1), std_430(:, component_index + 1), 'b.');
    ylabel(error_names{component_index});
    ylim([0, 0.08]);
    grid on;
    if component_index == 1
        title('各维度定位误差对比');
        legend('pva\_830', 'pva\_430');
    elseif component_index == 3
        xlabel('时间（s）');
    end
end
set(findall(error_figure, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
exportgraphics(error_figure, fullfile(figure_dir, ...
    'position-standard-deviation-comparison.png'), 'Resolution', 600);

%% PVA 数据快速检查
pva_figure = myfigurestartup(7, 5, 'paper');
subplot(1, 2, 1);
plot(diff(pva_830(:, 5)), '.');
title('pva\_830 第5列差分');
xlabel('历元');
grid on;
subplot(1, 2, 2);
plot(pva_830(:, 8));
title('pva\_830 第8列数据');
xlabel('历元');
grid on;
set(findall(pva_figure, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
exportgraphics(pva_figure, fullfile(figure_dir, ...
    'pva-830-data-check.png'), 'Resolution', 600);

%% 真实轨迹与三个固定信标
beacon_position = zeros(3, 3);
range_data = cell(3, 1);
for beacon_index = 1:3
    range_path = fullfile(input_dir, sprintf('range%d.txt', beacon_index));
    if ~isfile(range_path)
        error('请先运行 dataget.m 生成距离文件：%s', range_path);
    end
    range_data{beacon_index} = readmatrix(range_path, 'FileType', 'text');
    beacon_position(beacon_index, :) = range_data{beacon_index}(1, 4:6);
end
scene_figure = plot_trajectory_and_beacons(cfg.truthpath, beacon_position);
exportgraphics(scene_figure, fullfile(figure_dir, ...
    'trajectory-and-beacons.png'), 'Resolution', 600);
savefig(scene_figure, fullfile(figure_dir, ...
    'trajectory-and-beacons.fig'));

%% 三信标距离随时间变化
range_figure = myfigurestartup(12, 5, 'prese');
for beacon_index = 1:3
    subplot(1, 3, beacon_index);
    plot(range_data{beacon_index}(:, 1), ...
        range_data{beacon_index}(:, 2), 'LineWidth', 1.5);
    xlabel('时间（s）');
    ylabel('距离（m）');
    title(sprintf('信标 %d 距离', beacon_index));
    grid on;
end
set(findall(range_figure, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
exportgraphics(range_figure, fullfile(figure_dir, ...
    'three-beacon-range-series.png'), 'Resolution', 600);
savefig(range_figure, fullfile(figure_dir, ...
    'three-beacon-range-series.fig'));

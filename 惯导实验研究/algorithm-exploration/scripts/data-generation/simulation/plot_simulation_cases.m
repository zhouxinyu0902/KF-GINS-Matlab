function figure_paths = plot_simulation_cases(case_names)
%PLOT_SIMULATION_CASES 为已有仿真数据生成轨迹、信标和距离曲线图。
%   plot_simulation_cases() 绘制 case-00 至 case-04。
%   plot_simulation_cases("case-01") 只绘制指定场景。
%
% 图片按场景保存到 figures-tables/simulation/case-XX/input-figures。

    script_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(fileparts(fileparts(script_dir)));
    project_root = fileparts(fileparts(topic_dir));
    addpath(topic_dir);
    setup_inertial_experiment();
    glvs;

    if nargin < 1 || isempty(case_names)
        case_names = compose("case-%02d", 0:4);
    end
    if ischar(case_names)
        case_names = string({case_names});
    else
        case_names = string(case_names(:));
    end

    input_root = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration', 'input', 'simulation');
    figure_paths = strings(numel(case_names), 4);
    for case_index = 1:numel(case_names)
        case_name = char(case_names(case_index));
        case_dir = fullfile(input_root, case_name);
        figure_root = fullfile(project_root, 'data', ...
            'inertial-experiment', 'algorithm-exploration', ...
            'figures-tables', 'simulation', case_name, 'input-figures');
        if ~exist(figure_root, 'dir')
            mkdir(figure_root);
        end
        required_files = {
            fullfile(case_dir, 'truth.txt'), ...
            fullfile(case_dir, 'range1.txt'), ...
            fullfile(case_dir, 'range2.txt'), ...
            fullfile(case_dir, 'range3.txt')};
        for file_index = 1:numel(required_files)
            if ~isfile(required_files{file_index})
                error('场景 %s 缺少绘图输入：%s', ...
                    case_name, required_files{file_index});
            end
        end

        truth = readmatrix(required_files{1}, 'FileType', 'text');
        range_data = cell(3, 1);
        beacon_position = zeros(3, 3);
        for beacon_index = 1:3
            range_data{beacon_index} = readmatrix( ...
                required_files{beacon_index + 1}, 'FileType', 'text');
            beacon_position(beacon_index, :) = ...
                range_data{beacon_index}(1, 4:6);
        end

        % 真值位置为[纬度(deg), 经度(deg), 高度(m)]，信标为rad、rad、m。
        origin = [deg2rad(truth(1, 3:4))'; truth(1, 5)];
        truth_position = [deg2rad(truth(:, 3:4)), truth(:, 5)];
        trajectory_xyz = pos2dxyz(truth_position, origin);
        beacon_xyz = pos2dxyz(beacon_position, origin);

        % 复用专题现有的导航场景绘图函数。
        scene_figure = plot_navigation_scene(trajectory_xyz, ...
            'static', beacon_xyz, 'type', 'xyz', 'unit', 'km');
        set(scene_figure, 'Color', 'w', 'Name', ...
            [case_name, ' trajectory and beacons']);
        title(sprintf('%s：仿真轨迹与三信标位置', case_name));
        scene_png = fullfile(figure_root, ...
            [case_name, '-trajectory-beacons.png']);
        scene_fig = fullfile(figure_root, ...
            [case_name, '-trajectory-beacons.fig']);
        exportgraphics(scene_figure, scene_png, 'Resolution', 300);
        savefig(scene_figure, scene_fig);
        close(scene_figure);

        distance_figure = figure('Color', 'w', ...
            'Name', [case_name, ' beacon ranges'], ...
            'Position', [100, 80, 1250, 820]);
        layout = tiledlayout(distance_figure, 3, 1, ...
            'TileSpacing', 'compact', 'Padding', 'compact');
        title(layout, sprintf('%s：轨迹到三信标的水平距离', case_name), ...
            'FontWeight', 'bold');
        colors = lines(3);
        for beacon_index = 1:3
            nexttile(layout);
            plot(range_data{beacon_index}(:, 1), ...
                range_data{beacon_index}(:, 3) / 1000, ...
                'Color', colors(beacon_index, :), 'LineWidth', 1.2);
            grid on;
            box on;
            ylabel(sprintf('信标%d距离 (km)', beacon_index));
            if beacon_index == 3
                xlabel('时间 (s)');
            end
        end
        set(findall(distance_figure, '-property', 'FontName'), ...
            'FontName', 'Microsoft YaHei');
        distance_png = fullfile(figure_root, ...
            [case_name, '-beacon-ranges.png']);
        distance_fig = fullfile(figure_root, ...
            [case_name, '-beacon-ranges.fig']);
        exportgraphics(distance_figure, distance_png, 'Resolution', 300);
        savefig(distance_figure, distance_fig);
        close(distance_figure);

        figure_paths(case_index, :) = string({ ...
            scene_png, scene_fig, distance_png, distance_fig});
        fprintf('%s 图片已保存到：%s\n', case_name, figure_root);
    end
 end

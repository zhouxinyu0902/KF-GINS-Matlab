clear;
close all;

%% 路径与配置
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(script_dir);
project_root = fileparts(topic_dir);
cfg = patent_Configsimu();
if ~exist(cfg.evalfigurefolder, 'dir')
    mkdir(cfg.evalfigurefolder);
end

truthpath = cfg.truthpath;
new_output = cfg.outputfolder;
prefix = cfg.experiment.output_prefix;

%% 新结果：按测距间隔分别比较
pure_ins = fullfile(new_output, sprintf('%s-pure-ins.nav', prefix));

for interval_min = cfg.experiment.range_intervals_min
    max_candidates = 1 + numel(cfg.experiment.single_beacon_ids) + 1;
    candidates = cell(1, max_candidates);
    candidate_count = 0;

    if isfile(pure_ins)
        candidate_count = candidate_count + 1;
        candidates{candidate_count} = pure_ins;
    end

    if cfg.experiment.run_single_beacons
        for beacon_id = cfg.experiment.single_beacon_ids
            filename = sprintf('%s-range-single-b%02d-dt%s.nav', ...
                prefix, beacon_id, formatIntervalTag(interval_min));
            filepath = fullfile(new_output, filename);
            if isfile(filepath)
                candidate_count = candidate_count + 1;
                candidates{candidate_count} = filepath;
            else
                warning('缺少结果：%s', filepath);
            end
        end
    end

    if cfg.experiment.run_rotating_beacons
        sequence_tag = formatSequenceTag(cfg.experiment.rotation_sequence);
        filename = sprintf('%s-range-rotate-%s-dt%s.nav', ...
            prefix, sequence_tag, formatIntervalTag(interval_min));
        filepath = fullfile(new_output, filename);
        if isfile(filepath)
            candidate_count = candidate_count + 1;
            candidates{candidate_count} = filepath;
        else
            warning('缺少结果：%s', filepath);
        end
    end

    candidates = candidates(1:candidate_count);

    if isempty(candidates)
        warning('测距间隔 %s 没有可评价的新结果。', formatIntervalTag(interval_min));
        continue;
    end

    fprintf('评价测距间隔 %s，共 %d 个结果。\n', ...
        formatIntervalTag(interval_min), numel(candidates));
    interval_tag = formatIntervalTag(interval_min);
    plot_trj(truthpath, candidates{:});
    trajectory_fig = gcf;
    saveFigurePair(trajectory_fig, cfg.evalfigurefolder, ...
        ['evaluation-trajectory-dt', interval_tag], cfg.figure);

    [error_fig, ~] = calc_radial_error_gjb(truthpath, candidates{:});
    saveFigurePair(error_fig, cfg.evalfigurefolder, ...
        ['evaluation-radial-error-dt', interval_tag], cfg.figure);
end

%% 整理前历史结果（目录存在时只读对比）
legacy_output = cfg.legacyoutputfolder;
legacy_candidates = {
    fullfile(legacy_output, 'pureINS.nav'), ...
    fullfile(legacy_output, 'single1-4min.nav'), ...
    fullfile(legacy_output, 'single2-4min.nav'), ...
    fullfile(legacy_output, 'single3-4min.nav'), ...
    fullfile(legacy_output, '3-4min.nav')};
legacy_candidates = legacy_candidates(cellfun(@isfile, legacy_candidates));

if ~isempty(legacy_candidates)
    fprintf('评价整理前历史结果，共 %d 个文件。\n', numel(legacy_candidates));
    plot_trj(truthpath, legacy_candidates{:});
    trajectory_fig = gcf;
    saveFigurePair(trajectory_fig, cfg.evalfigurefolder, ...
        'evaluation-legacy-trajectory', cfg.figure);

    [error_fig, ~] = calc_radial_error_gjb(truthpath, legacy_candidates{:});
    saveFigurePair(error_fig, cfg.evalfigurefolder, ...
        'evaluation-legacy-radial-error', cfg.figure);
else
    fprintf('未找到整理前历史输出，跳过历史结果对比。\n');
end

%% 局部函数
function tag = formatSequenceTag(sequence)
    parts = arrayfun(@(id) sprintf('b%02d', id), sequence, 'UniformOutput', false);
    tag = strjoin(parts, '-');
end

function tag = formatIntervalTag(interval_min)
    seconds = interval_min * 60;
    if abs(seconds - round(seconds)) < 1e-9
        tag = sprintf('%ds', round(seconds));
    else
        value = regexprep(sprintf('%.3f', seconds), '0+$', '');
        value = regexprep(value, '\.$', '');
        tag = [strrep(value, '.', 'p'), 's'];
    end
end

function saveFigurePair(fig, output_folder, basename, figure_cfg)
    if ~isgraphics(fig, 'figure')
        warning('Cannot save figure %s because its handle is invalid.', basename);
        return;
    end
    if ~exist(output_folder, 'dir')
        mkdir(output_folder);
    end

    if figure_cfg.save_fig
        savefig(fig, fullfile(output_folder, [basename, '.fig']));
    end
    if figure_cfg.save_png
        png_path = fullfile(output_folder, [basename, '.png']);
        if exist('exportgraphics', 'file') == 2
            exportgraphics(fig, png_path, ...
                'Resolution', figure_cfg.png_resolution);
        else
            print(fig, png_path, '-dpng', ...
                sprintf('-r%d', figure_cfg.png_resolution));
        end
    end
end

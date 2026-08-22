clear;
close all;
clc;

%% GNSS/INS 探索结果评价
% 只读取 FGO_gnss_ins.m 已生成的导航文件，不重新运行组合导航。

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
setup_inertial_experiment();
cfg = ProcessConfig_exper();

gnss_ins_path = fullfile(cfg.outputfolder, 'GnssIns.nav');
if ~isfile(gnss_ins_path)
    error('缺少 GNSS/INS 结果，请先运行 FGO_gnss_ins.m：%s', ...
        gnss_ins_path);
end
if ~isfolder(cfg.figurefolder)
    mkdir(cfg.figurefolder);
end

calc_radial_error_gjb(cfg.truthpath, gnss_ins_path, true);
calc_error(gnss_ins_path, cfg.truthpath);

if isfile(cfg.pureinsfilepath)
    comparison_figure = calc_error(gnss_ins_path, cfg.pureinsfilepath);
    set(findall(comparison_figure, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    exportgraphics(comparison_figure, fullfile(cfg.figurefolder, ...
        'Radial-error.png'), 'Resolution', 600);
else
    warning('纯惯导结果不存在，跳过 GNSS/INS—纯惯导对比：%s', ...
        cfg.pureinsfilepath);
end

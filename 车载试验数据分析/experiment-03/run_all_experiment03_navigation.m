function run_all_experiment03_navigation(dataset_ids, unit_types)
%RUN_ALL_EXPERIMENT03_NAVIGATION 批跑PureINS、EKF、一次和二次RTS。
%
% 现有run_navigation_comparison.m仍是单批兼容入口。本函数通过环境
% 覆盖参数调用它，不删除旧入口，也不改变其默认运行方式。

    if nargin < 1 || isempty(dataset_ids)
        dataset_ids = 1:3;
    end
    if nargin < 2 || isempty(unit_types)
        unit_types = ["rad", "m"];
    end
    unit_types = lower(string(unit_types));
    if any(~ismember(unit_types, ["rad", "m"]))
        error('unit_types只能包含rad和m。');
    end

    script_dir = fileparts(mfilename('fullpath'));
    comparison_script = fullfile(script_dir, ...
        'run_navigation_comparison.m');
    if ~isfile(comparison_script)
        error('缺少兼容导航入口：%s', comparison_script);
    end

    previous_dataset = getenv('KF_GINS_EXPERIMENT03_DATASET');
    previous_unit = getenv('KF_GINS_EXPERIMENT03_UNIT');
    previous_methods = getenv('KF_GINS_EXPERIMENT03_METHODS');
    environment_cleanup = onCleanup(@() restore_environment( ...
        previous_dataset, previous_unit, previous_methods));

    for dataset_index = 1:numel(dataset_ids)
        paths = experiment03_dataset_paths(dataset_ids(dataset_index));
        if ~isfile(paths.truth_file)
            error('%s缺少truth.nav，请先运行build_dataset_truth_03。', ...
                paths.dataset_name);
        end
        imu_info = dir(paths.imu_120_file);
        truth_info = dir(paths.truth_file);
        if isempty(imu_info) || truth_info.datenum < imu_info.datenum
            error(['%s的truth.nav早于当前imu_120.txt，' ...
                '请重新运行build_dataset_truth_03。'], ...
                paths.dataset_name);
        end
        for unit_index = 1:numel(unit_types)
            unit = unit_types(unit_index);
            fprintf('\n============================================================\n');
            fprintf('批量导航：%s，%s单位\n', paths.dataset_name, unit);
            fprintf('============================================================\n');

            run_experiment03_pureins(paths.dataset_name, unit);

            setenv('KF_GINS_EXPERIMENT03_DATASET', paths.dataset_name);
            setenv('KF_GINS_EXPERIMENT03_UNIT', char(unit));
            setenv('KF_GINS_EXPERIMENT03_METHODS', 'ekf,rts1,rts2');
            command = sprintf('run(''%s'')', ...
                strrep(comparison_script, '''', ''''''));
            evalin('base', command);
        end
    end
    clear environment_cleanup;
    restore_environment(previous_dataset, previous_unit, previous_methods);
end

function restore_environment(dataset, unit, methods)
    setenv('KF_GINS_EXPERIMENT03_DATASET', dataset);
    setenv('KF_GINS_EXPERIMENT03_UNIT', unit);
    setenv('KF_GINS_EXPERIMENT03_METHODS', methods);
end

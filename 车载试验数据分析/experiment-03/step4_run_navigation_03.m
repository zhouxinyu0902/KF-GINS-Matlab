function step4_run_navigation_03(dataset_ids, unit_types)
%STEP4_RUN_NAVIGATION_03 批跑EKF、一次RTS和二次RTS。
%
% Pure INS由step2_run_pureins_03单独运行；本步骤仅执行正式组合导航。
% common/run_navigation_comparison.m是单批算法实现，本函数负责批次和单位调度。

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
    comparison_script = fullfile(script_dir, 'common', ...
        'run_navigation_comparison.m');
    if ~isfile(comparison_script)
        error('缺少正式导航核心：%s', comparison_script);
    end

    previous_dataset = getenv('KF_GINS_EXPERIMENT03_DATASET');
    previous_unit = getenv('KF_GINS_EXPERIMENT03_UNIT');
    previous_methods = getenv('KF_GINS_EXPERIMENT03_METHODS');
    environment_cleanup = onCleanup(@() restore_environment( ...
        previous_dataset, previous_unit, previous_methods));

    dataset_count = count_datasets(dataset_ids);
    for dataset_index = 1:dataset_count
        dataset_id = select_dataset(dataset_ids, dataset_index);
        paths = experiment03_dataset_paths(dataset_id);
        for unit_index = 1:numel(unit_types)
            unit = unit_types(unit_index);
            fprintf('\n============================================================\n');
            fprintf('Step 4 正式导航：%s，%s单位\n', ...
                paths.dataset_name, unit);
            fprintf('============================================================\n');

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

function count = count_datasets(dataset_ids)
    if ischar(dataset_ids), count = 1; else, count = numel(dataset_ids); end
end

function dataset_id = select_dataset(dataset_ids, index)
    if ischar(dataset_ids)
        dataset_id = dataset_ids;
    elseif iscell(dataset_ids)
        dataset_id = dataset_ids{index};
    else
        dataset_id = dataset_ids(index);
    end
end

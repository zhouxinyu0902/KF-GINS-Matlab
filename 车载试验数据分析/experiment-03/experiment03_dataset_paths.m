function paths = experiment03_dataset_paths(dataset_id)
%EXPERIMENT03_DATASET_PATHS 返回第三次试验各批次的统一目录。
% dataset_id 可取 1/2/3，或 run-0817/run-0818/run-0818-noon。

    if nargin < 1 || isempty(dataset_id)
        dataset_id = 'run-0817';
    end

    if isnumeric(dataset_id)
        if ~isscalar(dataset_id) || ~ismember(dataset_id, 1:3)
            error('数值型 dataset_id 只能取 1、2 或 3。');
        end
        dataset_key = sprintf('%d', dataset_id);
    else
        dataset_key = lower(strtrim(char(string(dataset_id))));
    end

    switch dataset_key
        case {'1', 'run-0817', '0817', 'data-0817'}
            dataset_name = 'run-0817';
            raw_mat_name = 'raw_data_0817.mat';
        case {'2', 'run-0818', '0818', 'data-0818'}
            dataset_name = 'run-0818';
            raw_mat_name = 'raw_data_0818.mat';
        case {'3', 'run-0818-noon', '0818-noon', 'data-0818-1'}
            dataset_name = 'run-0818-noon';
            raw_mat_name = 'raw_data_0818_1.mat';
        otherwise
            error('未知的第三次试验数据集：%s', char(string(dataset_id)));
    end

    topic_dir = fileparts(mfilename('fullpath'));
    analysis_root = fileparts(topic_dir);
    project_root = fileparts(analysis_root);
    experiment_root = fullfile(project_root, 'data', ...
        'experiment-data', 'experiment-03');

    paths.dataset_name = dataset_name;
    paths.topic = topic_dir;
    paths.analysis = analysis_root;
    paths.project = project_root;
    paths.experiment = experiment_root;
    paths.data = fullfile(experiment_root, dataset_name);
    paths.raw = fullfile(paths.data, 'raw');
    paths.intermediate = fullfile(paths.data, 'intermediate');
    paths.input = fullfile(paths.data, 'input');
    paths.output = fullfile(paths.data, 'output');
    paths.artifacts = fullfile(paths.output, 'artifacts');
    paths.raw_mat_file = fullfile(paths.intermediate, raw_mat_name);
end

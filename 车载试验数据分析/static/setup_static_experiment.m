function paths = setup_static_experiment(caseNo)
%SETUP_STATIC_EXPERIMENT Configure code, input, and output paths for static-003.

topic_root = fileparts(mfilename('fullpath'));
vehicle_analysis_root = fileparts(topic_root);
project_root = fileparts(vehicle_analysis_root);

% Add dependencies explicitly so the run does not depend on a saved MATLAB path.
addpath(fullfile(project_root, 'function'));
addpath(fullfile(project_root, 'function_zxy'));
addpath(fullfile(project_root, 'GINS-KF'));
addpath(topic_root);

paths.root = topic_root;
paths.project_root = project_root;
switch(caseNo)
    case 0
        paths.data_root = fullfile(project_root, 'data', 'experiment-data', 'static');
        paths.dataset_name = 'static-003';
    case 1
        paths.data_root = fullfile(project_root, 'data', 'experiment-data', 'static-1');
        paths.dataset_name = 'static-000';
end
paths.input = fullfile(paths.data_root, 'input', paths.dataset_name);
paths.output = fullfile(paths.data_root, 'output', paths.dataset_name, ...
    'navigation-results');
paths.artifacts = fullfile(paths.data_root, 'output', paths.dataset_name, ...
    'figures-tables');

paths.imu_file = fullfile(paths.input, 'IMU_120.txt');
paths.imu_alignment_file = fullfile(paths.input, 'IMU_120_static.txt');
paths.pva_initial_file = fullfile(paths.input, 'pva_initial.txt');
paths.pva_incomplete_file = fullfile(paths.input, 'pva_incomplete.txt');
paths.pureins_file = fullfile(paths.output, 'PureIns.nav');

required_files = {paths.imu_file, paths.pva_initial_file};
for k = 1:numel(required_files)
    if ~isfile(required_files{k})
        error('setup_static_experiment:MissingInput', ...
            'Required static-003 input file is missing: %s', required_files{k});
    end
end

required_directories = {paths.output, paths.artifacts};
for k = 1:numel(required_directories)
    if ~isfolder(required_directories{k})
        mkdir(required_directories{k});
    end
end

fprintf('Static experiment configured: %s\n', paths.dataset_name);
end

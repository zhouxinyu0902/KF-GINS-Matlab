clear;
close all;
clc;

%% 仿真数据生成入口
% 默认行为：
%   1）处理 case-00 至 case-04；
%   2）已有数据不覆盖，只生成缺失数据；
%   3）每次运行都会更新轨迹—信标图和三信标距离曲线图。
%
% 如需重新生成单个场景，可改为：
%   selected_cases = "case-01";
%   overwrite_existing = true;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));

addpath(topic_dir);
addpath(script_dir);
setup_inertial_experiment();

selected_cases = compose("case-%02d", 0:4);
overwrite_existing = false;

fprintf('开始处理仿真数据：%s\n', strjoin(selected_cases, ', '));
mode_messages = [ ...
    "保护模式：保留已有数据，只生成缺失数据并更新图片。", ...
    "覆盖模式：已有数据将被重新生成。"];
fprintf('%s\n', mode_messages(1 + overwrite_existing));

generated_cases = generate_simulation_data( ...
    selected_cases, overwrite_existing);

if isempty(generated_cases)
    fprintf('数据均已存在，本次只更新了输入数据图片。\n');
else
    fprintf('本次新生成或覆盖的场景：%s\n', ...
        strjoin(generated_cases, ', '));
end
fprintf('仿真数据处理完成。\n');

%% Step 1：生成典型轨迹及 DVL、罗盘仿真数据
% 本脚本只负责构造后续研究需要的数据，不进行误差分析和滤波。
%
% 统一参数：
%   采样周期             0.5 s
%   巡航速度             1 m/s
%   初始深度             100 m
%   DVL 刻度因子误差     正负 0.4%
%   罗盘航向系统误差     正负 0.23°
%   随机噪声             暂不加入
%
% 每个轨迹文件直接保存以下主要变量：
%   avp_ref               真实姿态、速度、位置和时间
%   local_xy              真实局部东、北位置，单位 m
%   dvl_true              真实 DVL 水平速度
%   dvl_plus / dvl_minus  带正负 0.4% 刻度误差的 DVL 速度
%   yaw_true              真实航向
%   yaw_plus / yaw_minus  带正负 0.23°误差的罗盘航向
%   depth_true            真实深度

clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
psins_root = fileparts(fileparts(fileparts(study_dir)));
addpath(genpath(psins_root));
glvs;

%% 仿真参数
ts = 0.5;
cruise_speed_mps = 1.0;
acceleration_time_s = 20;
initial_static_time_s = 10;
final_static_time_s = 10;
turn_rate_dps = 1.0;
initial_yaw_deg = 180;
initial_depth_m = 100;
dvl_scale_error = 0.004;
compass_error_deg = 0.23;

scenario_ids = {'straight', 'single_turn', 'rectangle', 's_turn'};
scenario_names_zh = {'直线航迹', '单次 90° 转弯', '矩形航迹', 'S 形航迹'};
scenario_descriptions = { ...
    '直线巡航，用于观察误差线性累积和单距离横向弱约束。', ...
    '单次转弯，用于观察航向变化带来的几何改善。', ...
    '矩形航迹，用于观察多次视线方向变化。', ...
    'S 形航迹，用于观察连续时变的视线几何。'};
file_names = { ...
    'trajectory_01_straight.mat', ...
    'trajectory_02_single_turn.mat', ...
    'trajectory_03_rectangle.mat', ...
    'trajectory_04_s_turn.mat'};

output_dir = fullfile(study_dir, 'generated_data');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

pos0 = glv.pos0;
pos0(3) = -initial_depth_m;
avp0 = [[0; 0; initial_yaw_deg * glv.deg]; [0; 0; 0]; pos0];

duration_s = zeros(numel(file_names), 1);
sample_count = zeros(numel(file_names), 1);
path_length_m = zeros(numel(file_names), 1);

figure('Name', '典型仿真轨迹', 'Color', 'w');
hold on;
grid on;
axis equal;
colors = lines(numel(file_names));

for k = 1:numel(file_names)
    seg = build_scenario(scenario_ids{k}, cruise_speed_mps, ...
        acceleration_time_s, initial_static_time_s, ...
        final_static_time_s, turn_rate_dps);
    trj = trjsimu(avp0, seg.wat, ts, 1);

    avp_ref = trj.avp;
    time_s = avp_ref(:, end);
    local_xyz = pos2dxyz(avp_ref(:, 7:9), avp_ref(1, 7:9)');
    local_xy = local_xyz(:, 1:2);
    yaw_true = avp_ref(:, 3);
    yaw_plus = wrap_pi(yaw_true + compass_error_deg * glv.deg);
    yaw_minus = wrap_pi(yaw_true - compass_error_deg * glv.deg);
    depth_true = avp_ref(:, 9);

    dvl_true = zeros(size(avp_ref, 1), 2);
    for i = 1:size(avp_ref, 1)
        cnb = a2mat(avp_ref(i, 1:3));
        velocity_body = cnb' * avp_ref(i, 4:6)';
        dvl_true(i, :) = velocity_body(1:2)';
    end
    dvl_plus = (1 + dvl_scale_error) * dvl_true;
    dvl_minus = (1 - dvl_scale_error) * dvl_true;

    duration_s(k) = time_s(end) - time_s(1);
    sample_count(k) = numel(time_s);
    path_length_m(k) = sum(sqrt(sum(diff(local_xy).^2, 2)));

    save(fullfile(output_dir, file_names{k}), ...
        'trj', 'avp_ref', 'time_s', 'local_xy', ...
        'dvl_true', 'dvl_plus', 'dvl_minus', ...
        'yaw_true', 'yaw_plus', 'yaw_minus', 'depth_true', ...
        'ts', 'dvl_scale_error', 'compass_error_deg');

    plot(local_xy(:, 1), local_xy(:, 2), ...
        'LineWidth', 1.3, 'Color', colors(k, :), ...
        'DisplayName', scenario_names_zh{k});
end

xlabel('东向 / m');
ylabel('北向 / m');
title('DVL + 罗盘航位推算典型轨迹');
legend('Location', 'best');

save(fullfile(output_dir, 'trajectory_catalog.mat'), ...
    'scenario_ids', 'scenario_names_zh', 'scenario_descriptions', ...
    'file_names', 'duration_s', 'sample_count', 'path_length_m', ...
    'ts', 'cruise_speed_mps', 'dvl_scale_error', 'compass_error_deg');

fprintf('\n已生成真实轨迹、DVL 和罗盘数据，保存位置：\n%s\n', output_dir);
fprintf('DVL 误差：正负 %.1f%%；罗盘误差：正负 %.2f°。\n', ...
    100 * dvl_scale_error, compass_error_deg);


function seg = build_scenario(name, speed, acceleration_time, ...
        initial_static_time, final_static_time, turn_rate)
% 根据场景名称构造运动段。
    acceleration = speed / acceleration_time;
    turn_time_90 = 90 / turn_rate;

    seg = trjsegment([], 'init', 0);
    seg = trjsegment(seg, 'uniform', initial_static_time);
    seg = trjsegment(seg, 'accelerate', acceleration_time, [], acceleration);

    switch name
        case 'straight'
            seg = trjsegment(seg, 'uniform', 1800);
        case 'single_turn'
            seg = trjsegment(seg, 'uniform', 900);
            seg = trjsegment(seg, 'turnleft', turn_time_90, turn_rate);
            seg = trjsegment(seg, 'uniform', 900);
        case 'rectangle'
            for side = 1:4
                seg = trjsegment(seg, 'uniform', 600);
                if side < 4
                    seg = trjsegment(seg, 'turnleft', turn_time_90, turn_rate);
                end
            end
        case 's_turn'
            seg = trjsegment(seg, 'uniform', 500);
            seg = trjsegment(seg, 'turnright', turn_time_90, turn_rate);
            seg = trjsegment(seg, 'uniform', 600);
            seg = trjsegment(seg, 'turnleft', 2 * turn_time_90, turn_rate);
            seg = trjsegment(seg, 'uniform', 600);
            seg = trjsegment(seg, 'turnright', turn_time_90, turn_rate);
            seg = trjsegment(seg, 'uniform', 500);
        otherwise
            error('未知轨迹场景：%s', name);
    end

    seg = trjsegment(seg, 'deaccelerate', acceleration_time, [], acceleration);
    seg = trjsegment(seg, 'uniform', final_static_time);
end


function angle = wrap_pi(angle)
% 将角度限制在 [-pi, pi]。
    angle = atan2(sin(angle), cos(angle));
end

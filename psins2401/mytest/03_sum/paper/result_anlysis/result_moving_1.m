% FourStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_moving_4state.mat');
% FiveStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_moving_5state.mat');
% path11 = 'D:\WPS云盘\469639050\WPS云盘\成果\1_DR_RANGE\figs\';
path11 =  'D:\Github\KF-GINS-Matlab\data\psins\figures\';
% FourStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_moving_4state.mat');
% FiveStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_moving_5state.mat');
path = 'D:\Github\KF-GINS-Matlab\data\psins\datasaved_new';
FourStatebeacon = load([path,'\data_exper_moving_4state.mat']);
FiveStatebeacon = load([path,'\data_exper_moving_5state.mat']);
%% ============================================================
% 同时比较 4-State 和 5-State 下两个移动信标 B9 / B10
% 逻辑：
% DR
% 4-State-B9
% 5-State-B9
% 4-State-B10
% 5-State-B10
% ============================================================
% prefix = 'simu-moving-state45-';
prefix = 'exper-moving-state45-';
target_states = [4]; % 同时比较 4-State 和 5-State

moving_idx = 1; % B9 和 B10 为两个移动信标
% ==============================
% 1. 初始化容器
% ==============================
all_aided_avps = {};
all_beacon_pos = {};
all_XK = {};
all_labels = {'DR'}; % labels 第一个必须是 DR
% ==============================
% 2. 按“移动信标优先”的顺序组织数据
% 这样 B9 下的 4-State / 5-State 会挨在一起
% ==============================

for i = 1:length(moving_idx)
    idx = moving_idx(i);
    for s = 1:length(target_states)
        target_state = target_states(s);
        switch target_state
            case 4
                S = FourStatebeacon;
                model_name = '4-State';
            case 5
                S = FiveStatebeacon;
                model_name = '5-State';
            case 7
                S = SevenStatebeacon;
                model_name = '7-State';
            otherwise
                error('target_state must be 4, 5, or 7.');
        end
        % ==============================
        % 2.1 提取导航结果
        % ==============================
        avp_i = S.avp_range{idx};
        % ==============================
        % 2.2 提取状态估计结果
        % ==============================
        xk_i = S.XK{1, idx};
        % ==============================
        % 2.3 提取移动信标位置
        % 注意：B9 对应 moving_beacons，B10 对应 moving_beacons1
        % ==============================
        beacon_pos_i = S.moving_beacons.pos{idx};

        % ==============================
        % 2.4 长度对齐
        % 避免 avp_i、xk_i、beacon_pos_i 长度不一致
        % ==============================
        len_min = min([ ...
            size(avp_i, 1), ...
            size(xk_i, 1), ...
            size(beacon_pos_i, 1)]);
        xk_i = xk_i(1:len_min, :);
        beacon_pos_i = beacon_pos_i(1:len_min, :);
        % ==============================
        % 2.5 存入统一容器
        % 重要：
        % all_aided_avps{k}
        % all_XK{k}
        % all_beacon_pos{k}
        % all_labels{k+1}
        % 必须一一对应
        % ==============================
        all_aided_avps{end+1} = avp_i;
        all_XK{end+1} = xk_i;
        all_beacon_pos{end+1} = beacon_pos_i;
        all_labels{end+1} = sprintf('%s-B%d', model_name, idx);
    end
end
% ==============================
% 3. 检查数组逻辑是否一致
% ==============================
num_cases = length(all_aided_avps);
if length(all_XK) ~= num_cases || length(all_beacon_pos) ~= num_cases
    error('all_aided_avps、all_XK、all_beacon_pos 数量不一致。');
end
if length(all_labels) ~= num_cases + 1
    error('all_labels 数量错误：第一个应为 DR，后面应与 all_aided_avps 一一对应。');
end
disp('===== 当前对比标签顺序 =====');
disp(all_labels(:));
% ==============================
% 4. 统一参考轨迹和 DR
% 这里假设 4-State 和 5-State 使用相同的 avp_ref / avp_dr
% 如果不完全相同，建议以 FourStatebeacon 为统一基准
% ==============================
base_struct = FourStatebeacon;
ref = base_struct.avp_ref;
dr = base_struct.avp_dr;
dk = base_struct.dk;
dphi = base_struct.dphi_deg;
dt = base_struct.dt;
% ==============================
% 5. 调用理论-实际误差分析函数
% ==============================
H = analyze_theory_vs_actual_error_moving_beacon_v2( ...
    ref, ...
    dr, ...
    all_aided_avps, ...
    prefix, ...
    all_XK, ...
    all_beacon_pos, ...
    all_labels, ...
    dk, ...
    dphi, ...
    dt);
% ==============================
% 6. 保存 analyze 函数生成的图
% ==============================
kk = [1, 2, 4];
kk = kk(kk <= length(H.all_axes));
for k = kk
    file_path = fullfile(path11, [H.prefix, H.all_names{k}, '.png']);
    exportgraphics(H.all_axes(k), file_path, 'Resolution', 600);
end
%% ==============================
% 7. 计算并绘制径向误差
% 注意：这里直接使用 all_aided_avps
% 不要再重新用 current_struct.avp_range(moving_idx)
% 否则只会取到单一 state 的结果
% ==============================
[radial_errors, stats] = calc_radial_error_avp( ...
    ref, ...
    all_labels, ...
    dr, ...
    all_aided_avps{:});

exportgraphics(gcf, ...
    fullfile(path11, [prefix, 'radial-error.png']), ...
    'Resolution', 600);
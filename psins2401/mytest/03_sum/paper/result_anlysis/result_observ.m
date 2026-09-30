clear
path11 = 'D:\WPS云盘\469639050\WPS云盘\成果\1_DR_RANGE\figs\';
IsExp = 0;
if  IsExp ==0
    FourStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_simu_4state.mat');
    FiveStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_simu_5state.mat');
    SevenStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_simu_7state.mat');
    beacon_pos_cell = {FourStatebeacon.beacon_data.fixed_pos(1,:),...
        FourStatebeacon.beacon_data.fixed_pos(2,:),...
        FourStatebeacon.beacon_data.fixed_pos(3,:),...
        FourStatebeacon.beacon_data.fixed_pos(4,:),...
        FourStatebeacon.moving_beacons.pos,...
        FourStatebeacon.moving_beacons1.pos};
    labels = {'simu trajectory','beacon 1','beacon 2','beacon 3','beacon 4','moving beacon 1(B9)','moving beacon 2(B10)'};

    % fig = plot_beacon_comparison_v2(FourStatebeacon.avp_ref(:,7:9), beacon_pos_cell, labels);
    % ylim([-3000 1500])
    % exportgraphics(fig, fullfile('', [path11,'trj_vs_bea_square.png']), 'Resolution', 600);

    fig = plot_beacon_comparison_v2(FourStatebeacon.avp_ref(:,7:9), beacon_pos_cell(1:4), labels(1:5));
    ylim([-3000 1500])
    exportgraphics(fig, fullfile('', [path11,'trj_vs_bea_square_fixed.png']), 'Resolution', 600);

    fig = plot_beacon_comparison_v2(FourStatebeacon.avp_ref(:,7:9), beacon_pos_cell(5:6), labels([1,6:7]));
    ylim([-2000 1500])
    exportgraphics(fig, fullfile('', [path11,'trj_vs_bea_square_moving.png']), 'Resolution', 600);

else
    FourStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_4state.mat');
    FiveStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_5state.mat');
    SevenStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_7state.mat');
    beacon_pos_cell = {FourStatebeacon.beacon_data.fixed_pos(1,:),...
        FourStatebeacon.beacon_data.fixed_pos(3,:)};
    labels = {'real trajectory','beacon 1','beacon 3'};
    fig = plot_beacon_comparison_v2(FourStatebeacon.avp_ref(:,7:9), beacon_pos_cell, labels);
    ylim([-1500 1500])
    exportgraphics(gcf, fullfile('', [path11,'trj_vs_bea_b13.png']), 'Resolution', 600);

    beacon_pos_cell = {FourStatebeacon.beacon_data.fixed_pos(1,:),...
        FourStatebeacon.beacon_data.fixed_pos(2,:),...
        FourStatebeacon.beacon_data.fixed_pos(3,:),...
        FourStatebeacon.beacon_data.fixed_pos(4,:),...
        FourStatebeacon.moving_beacons.pos};
    labels = {'real trajectory','beacon 1','beacon 2','beacon 3','beacon 4','moving beacon'};
    fig1 = plot_beacon_comparison_v2(FourStatebeacon.avp_ref(:,7:9), beacon_pos_cell, labels);
    ylim([-3000 1500])
    exportgraphics(fig1, fullfile('', [path11,'real-trj_vs_bea_square.png']), 'Resolution', 600);
    beacon_pos_cell = {FourStatebeacon.beacon_data.fixed_pos(1,:),...
        FourStatebeacon.beacon_data.fixed_pos(3,:)};
end

%%
for state =[4,5,7]
    if state == 5
        current_struct = FiveStatebeacon;
        model_name = '5-State';
    elseif state == 4
        current_struct = FourStatebeacon;
        model_name = '4-State';
    elseif state == 7
        current_struct = SevenStatebeacon;
        model_name = '7-State';
    end

    bbb = 1:4; % 选择信标范围

    % 统一提取
    aided_avps = current_struct.avp_range(bbb);
    HHks = current_struct.HHk(bbb);
    beacon_pos = num2cell(current_struct.beacon_data.fixed_pos(bbb,:), 2)';
    XK_1 = current_struct.XK(1, bbb);
    labels = [{'DR'}, arrayfun(@(x) sprintf('%s-B%d', model_name, x), bbb, 'UniformOutput', false)];
    calc_radial_error_avp(current_struct.avp_ref,labels,current_struct.avp_dr,aided_avps{:});
end
%% 3D轨迹
avp_ref = FourStatebeacon.avp_ref;
lat_ref = avp_ref(:, 7);     % latitude, rad
lon_ref = avp_ref(:, 8);     % longitude, rad
dep_ref = avp_ref(:, 9);     % depth, m
lat0 = lat_ref(1);
lon0 = lon_ref(1);
Re = 6378137;
E_ref = (lon_ref - lon0) .* Re .* cos(lat0);
N_ref = (lat_ref - lat0) .* Re;
D_ref = dep_ref;
fig2 = myfigurestartup(4, 4, 'zxy');
set(fig2, 'Name', 'Reference 3D Trajectory');
plot3(E_ref, N_ref, D_ref, 'k-', 'LineWidth', 2);
hold on;
plot3(E_ref(1), N_ref(1), D_ref(1), 'go', ...
    'MarkerSize', 7, 'LineWidth', 1.5, ...
    'DisplayName', 'Start');

plot3(E_ref(end), N_ref(end), D_ref(end), 'ro', ...
    'MarkerSize', 7, 'LineWidth', 1.5, ...
    'DisplayName', 'End');
grid on;
box on;
% axis equal;
xlabel('East (m)');
ylabel('North (m)');
zlabel('Depth (m)');
legend('Reference trajectory', 'Start', 'End', 'Location', 'best');
view(45, 25);
exportgraphics(gcf, fullfile('', [path11,'3D.png']), 'Resolution', 600);

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
target_states = [4, 5]; % 同时比较 4-State 和 5-State

moving_idx = [9]; % B9 和 B10 为两个移动信标
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
        switch idx
            case 9
                beacon_pos_i = S.moving_beacons.pos;
            case 10
                beacon_pos_i = S.moving_beacons1.pos;
            otherwise

                error('当前代码只适用于移动信标 B9 和 B10。');
        end
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
% ==============================
% 7. 计算并绘制径向误差
% 注意：这里直接使用 all_aided_avps
% 不要再重新用 current_struct.avp_range(moving_idx)
% 否则只会取到单一 state 的结果
% ==============================
% [radial_errors, stats] = calc_radial_error_avp( ...
%     ref, ...
%     all_labels, ...
%     dr, ...
%     all_aided_avps{:});
% exportgraphics(gcf, ...
%     fullfile(path11, [prefix, 'radial-error.png']), ...
%     'Resolution', 600);
%%
close all
% 定义你要对比的模型结构体和名称
target_models = {FourStatebeacon, FiveStatebeacon,SevenStatebeacon};
model_tags = {'4-State', '5-State','7-state'};

prefix = 'simu-';
% prefix = 'exper-';
if   IsExp ==0
    bbb = [1,3]; % 假设只选前两个信标进行跨模型对比，防止图线太多太乱
    prefix = 'model-B13-';
    kk = [2,4];
else
    bbb = [1,3]; % 假设只选前两个信标进行跨模型对比，防止图线太多太乱
    prefix = 'exper-B13-';
    kk =[2,4];
end
% bbb = 1:4;
% 初始化用于绘图的容器
all_aided_avps = {};
all_HHks = {};
all_beacon_pos = {};
all_XK = {};
all_labels = {'DR'}; % 基础对比线
for i = 1:length(bbb)
    for m = 1:length(target_models)
        S = target_models{m};
        name = model_tags{m};
        idx = bbb(i);
        all_aided_avps{end+1} = S.avp_range{idx};
        all_HHks{end+1} = S.HHk{idx};
        all_beacon_pos{end+1} = S.beacon_data.fixed_pos(idx,:);
        all_XK{end+1} = S.XK{1, idx};
        all_labels{end+1} = sprintf('%s-B%d', name, idx);  % 动态生成标签，例如 "4-State-B1"
    end
end


ref = FiveStatebeacon.avp_ref;
dr = FiveStatebeacon.avp_dr;
dk = FiveStatebeacon.dk;
dphi = FiveStatebeacon.dphi_deg;
dt = FiveStatebeacon.dt;

% 调用分析函数
[radial_errors, stats] = calc_radial_error_avp(ref,all_labels, ...
    dr,all_aided_avps{:});

H = analyze_theory_vs_actual_error_moving_beacon_v2(ref, dr, all_aided_avps, ...
    prefix, all_XK, all_beacon_pos, all_labels, dk, dphi, dt);

for k = kk
    file_path = fullfile(path11, [H.prefix, H.all_names{k}, '.png']);
    exportgraphics(H.all_axes(k), file_path, ...
        'Resolution', 600);
end

% exportgraphics(H.figA, fullfile(export_folder, [H.prefix, 'Preview_Figure_A.png']), ...
%     'Resolution', 600);
%
% exportgraphics(H.figB, fullfile(export_folder, [H.prefix, 'Preview_Figure_B.png']), ...
%     'Resolution', 600);
%%
close all
% 定义你要对比的模型结构体和名称
target_models = {FourStatebeacon, FiveStatebeacon,SevenStatebeacon};
model_tags = {'4-State', '5-State','7-state'};

prefix = 'all';
bbb = 1:4;
% 初始化用于绘图的容器
all_aided_avps = {};
all_HHks = {};
all_beacon_pos = {};
all_XK = {};
all_labels = {'DR'}; % 基础对比线
for i = 1:length(bbb)
    for m = 1:length(target_models)
        S = target_models{m};
        name = model_tags{m};
        idx = bbb(i);
        all_aided_avps{end+1} = S.avp_range{idx};
        all_HHks{end+1} = S.HHk{idx};
        all_beacon_pos{end+1} = S.beacon_data.fixed_pos(idx,:);
        all_XK{end+1} = S.XK{1, idx};
        all_labels{end+1} = sprintf('%s-B%d', name, idx);  % 动态生成标签，例如 "4-State-B1"
    end
end

ref = FiveStatebeacon.avp_ref;
dr = FiveStatebeacon.avp_dr;


% 调用分析函数
[radial_errors, stats] = calc_radial_error_avp(ref,all_labels, ...
    dr,all_aided_avps{:});
%% 方案二：同模型下不同信标对比 (封装版)
% 只需要修改这一行，即可切换看 4维 还是 5维
% current_struct = FourStatebeacon;
% model_name = '4-State';
current_struct = FiveStatebeacon;
model_name = '5-State';


% bbb = [2,6,4,8]; % 选择信标范围
bbb = [1,5,2,6]; % 选择信标范围
% bbb = [1,5,4,8]; % 选择信标范围
if   IsExp ==0
    prefix = 'observ-B2468-';
    kk=[2,4];
else
    prefix = 'exper-B2468-';
    kk=4;
end
% bbb = 1:4;
% 统一提取
aided_avps = current_struct.avp_range(bbb);
HHks = current_struct.HHk(bbb);
beacon_pos = num2cell(current_struct.beacon_data.fixed_pos(bbb,:), 2)';
XK_1 = current_struct.XK(1, bbb);
labels = [{'DR'}, arrayfun(@(x) sprintf('%s-B%d', model_name, x), bbb, 'UniformOutput', false)];


calc_radial_error_avp(current_struct.avp_ref,labels,current_struct.avp_dr,aided_avps{:});
% 执行分析
H = analyze_theory_vs_actual_error_moving_beacon_v2(current_struct.avp_ref, ...
    current_struct.avp_dr, aided_avps, prefix, XK_1, beacon_pos, labels, ...
    current_struct.dk, current_struct.dphi_deg, current_struct.dt);

for k = kk
    file_path = fullfile(path11, [H.prefix, H.all_names{k}, '.png']);
    exportgraphics(H.all_axes(k), file_path, ...
        'Resolution', 600);
end


%%

function fig = plot_beacon_comparison_v2(pos_ref, beacon_pos_cell, labels)
% plot_beacon_comparison_v2: 绘制载体轨迹与信标（静止/移动）的空间分布对比图
% 输入:
%   pos_ref: 载体参考轨迹 [Lat, Lon, Alt] (单位: rad)
%   beacon_pos_cell: 包含信标位置的 cell，每个元素为 [Lat, Lon] 或 [Lat_seq, Lon_seq]
%   labels: 字符串 cell, 第一个为轨迹标签，后续为各信标标签

% --- 初始化参数 ---
Re = 6378137; % 地球长半径
colors_paper =[
    0.051, 0.251, 0.502;
    0.651, 0.102, 0.153;
    0.000, 0.600, 0.498;
    1.000, 0.498, 0.055;
    0.337, 0.706, 0.914;
    0.902, 0.624, 0.000;
    0.800, 0.475, 0.655;
    0.000, 0.447, 0.698
    ];
% --- 创建画布 ---
% 使用你要求的 myfigurestartup(5, 3, 'paper')
fig = myfigurestartup(5, 3, 'zxy');
hold on; grid on; box on;

% --- 坐标转换 (以轨迹起点为原点投影至投影平面) ---
ref_L0 = pos_ref(1,1);
ref_lam0 = pos_ref(1,2);

% 载体轨迹投影
x_ref = (pos_ref(:,2) - ref_lam0) * Re * cos(ref_L0);
y_ref = (pos_ref(:,1) - ref_L0) * Re;

% 绘制参考轨迹
plot(x_ref, y_ref, 'k-', 'LineWidth', 2, 'DisplayName', labels{1});
plot(x_ref(1), y_ref(1), '*', 'LineWidth', 2, 'DisplayName', 'start point');
% --- 绘制信标 ---
num_beacons = length(beacon_pos_cell);
for i = 1:num_beacons
    b_p = beacon_pos_cell{i};
    color = colors_paper(mod(i-1, size(colors_paper,1)) + 1, :);

    % 信标坐标投影
    xb = (b_p(:,2) - ref_lam0) * Re * cos(ref_L0);
    yb = (b_p(:,1) - ref_L0) * Re;

    if size(b_p, 1) == 1
        % 情况 A: 静止信标 (用五角星表示)
        plot(xb, yb, 'p', 'MarkerSize', 10, 'MarkerFaceColor', color, ...
            'Color', color, 'DisplayName', labels{i+1});
        text(xb-100, yb-100, sprintf("B%d",i), ...
            'VerticalAlignment', 'top', ... % 文字位于标记下方，也可选 'top', 'middle'
            'HorizontalAlignment', 'center', ... % 文字水平居中
            'FontSize', 8, ...                  % 可选：设置字体大小
            'Color', color);                    % 可选：设置文字颜色与标记一致
    else
        % 情况 B: 移动信标 (用虚线表示)
        plot(xb, yb, '--', 'LineWidth', 1, 'Color', color, ...
            'DisplayName', labels{i+1});
        % 起点标记
        plot(xb(1), yb(1), 'o', 'MarkerSize', 4, 'Color', color, 'HandleVisibility', 'off');
    end
end

% --- 图形修饰 ---
axis equal; % 保持等比例，防止地理形状畸变
xlabel('East (m)');
ylabel('North (m)');
% title('Space Geometry: Beacon vs Trajectory');

% 图例放在右侧 (eastoutside)
legend('Location', 'eastoutside');

% 适当紧缩边距
% set(gca, 'LooseInset', get(gca, 'TightInset'));

ax = gca;               % 获取当前坐标轴
ax.XAxis.Exponent = 0;  % 取消横坐标科学计数法
% 如果纵坐标也有同样问题，可以加上：
% ax.YAxis.Exponent = 0;
end
function ang = wrapTo180_local(ang)
% 将角度限制到 [-180, 180]
ang = mod(ang + 180, 360) - 180;
end
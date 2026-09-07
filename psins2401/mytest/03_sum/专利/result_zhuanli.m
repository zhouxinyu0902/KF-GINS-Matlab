clear
path11 = 'D:\Github\PSINS\psins2401\mytest\03_sum\专利\';
if ~exist(path11,'dir')
    mkdir(path11)
end
FourStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_simu_4state.mat');
FiveStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_simu_5state.mat');
SevenStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_simu_7state.mat');

beacon_pos_cell = {FourStatebeacon.beacon_data.fixed_pos(1,:),...
    FourStatebeacon.beacon_data.fixed_pos(2,:),...
    FourStatebeacon.beacon_data.fixed_pos(3,:),...
    FourStatebeacon.beacon_data.fixed_pos(4,:),...
    FourStatebeacon.moving_beacons.pos,...
    FourStatebeacon.moving_beacons1.pos};
labels = {'simu trajectory','moving beacon'};
fig = plot_beacon_comparison_v2(FourStatebeacon.avp_ref(:,7:9), beacon_pos_cell(5), labels);
legend('轨迹','起点','移动信标')
ylim([-3000 2000])
xygo('东向（m）','北向（m）')
exportpngandpdf(fig, fullfile('', [path11,'trj_vs_bea_square']));
% 移动信标
prefix = 'simu-moving-state457-';
target_states = [4, 5]; % 同时比较 4-State 和 5-State
moving_idx = [9,10]; % B9 和 B10 为两个移动信标
moving_idx = 9;
all_aided_avps = {};
all_beacon_pos = {};
all_XK = {};
all_labels = {'DR'}; % labels 第一个必须是 DR

for i = 1:length(moving_idx)
    idx = moving_idx(i);
    for s = 1:length(target_states)
        target_state = target_states(s);
        switch target_state
            case 4
                S = FourStatebeacon;
                model_name = '四维恒定航向偏差模型';
            case 5
                S = FiveStatebeacon;
                model_name = '本发明五维模型';
            case 7
                S = SevenStatebeacon;
                model_name = '7-State';
            otherwise
                error('target_state must be 4, 5, or 7.');
        end
        avp_i = S.avp_range{idx};
        xk_i = S.XK{1, idx};
        switch idx
            case 9
                beacon_pos_i = S.moving_beacons.pos;
            case 10
                beacon_pos_i = S.moving_beacons1.pos;
            otherwise
                error('当前代码只适用于移动信标 B9 和 B10。');
        end
        len_min = min([ ...
            size(avp_i, 1), ...
            size(xk_i, 1), ...
            size(beacon_pos_i, 1)]);
        xk_i = xk_i(1:len_min, :);
        beacon_pos_i = beacon_pos_i(1:len_min, :);
        all_aided_avps{end+1} = avp_i;
        all_XK{end+1} = xk_i;
        all_beacon_pos{end+1} = beacon_pos_i;
        all_labels{end+1} = sprintf('%s-移动信标', model_name);
    end
end
num_cases = length(all_aided_avps);
if length(all_XK) ~= num_cases || length(all_beacon_pos) ~= num_cases
    error('all_aided_avps、all_XK、all_beacon_pos 数量不一致。');
end
if length(all_labels) ~= num_cases + 1
    error('all_labels 数量错误：第一个应为 DR，后面应与 all_aided_avps 一一对应。');
end
disp('===== 当前对比标签顺序 =====');
disp(all_labels(:));
base_struct = FourStatebeacon;
ref = base_struct.avp_ref;
dr = base_struct.avp_dr;
dk = base_struct.dk;
dphi = base_struct.dphi_deg;
dt = base_struct.dt;
% if length(moving_idx)==2
calc_radial_error_avp(ref, all_labels,dr,all_aided_avps{:});
axis([0 8800 0 100])
xygo('时间（s）','水平位置误差（m）')
exportpngandpdf(gca, fullfile(path11, '移动信标'));
% exportgraphics(gcf, ...
%     fullfile(path11, [prefix, 'radial-error.png']), ...
%     'Resolution', 600);
% else
%     H = analyze_theory_vs_actual_error_moving_beacon_v2( ...
%         ref, ...
%         dr, ...
%         all_aided_avps, ...
%         prefix, ...
%         all_XK, ...
%         all_beacon_pos, ...
%         all_labels, ...
%         dk, ...
%         dphi, ...
%         dt);
%     % ==============================
%     % 6. 保存 analyze 函数生成的图
%     % ==============================
%     kk = [2, 4];
%     kk = kk(kk <= length(H.all_axes));
%     for k = kk
%         exportpngandpdf(H.all_axes(k), fullfile(path11, [H.prefix, H.all_names{k}]));
%     end
% end
%%
clear
path11 = 'D:\Github\PSINS\psins2401\mytest\03_sum\专利\';
if ~exist(path11,'dir')
    mkdir(path11)
end
FourStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_4state.mat');
FiveStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_5state.mat');
SevenStatebeacon = load('D:\GitHub\PSINS\psins2401\mytest\03_sum\datasaved_new\data_exper_7state.mat');


beacon_pos_cell = {FourStatebeacon.beacon_data.fixed_pos(1,:),...
    FourStatebeacon.beacon_data.fixed_pos(2,:),...
    FourStatebeacon.beacon_data.fixed_pos(3,:),...
    FourStatebeacon.beacon_data.fixed_pos(4,:),...
    FourStatebeacon.moving_beacons.pos};
labels = {'simu trajectory','1'};
fig = plot_beacon_comparison_v2(FourStatebeacon.avp_ref(:,7:9), beacon_pos_cell(1), labels);
legend('实测轨迹','起点','固定信标')
ylim([-1500 1000])
xygo('东向（m）','北向（m）')
exportpngandpdf(fig, fullfile('', [path11,'exper-trj_vs_bea_square']));

target_models = {FourStatebeacon, FiveStatebeacon};
model_tags = {'4-State', '5-State'};

bbb = 1; % 假设只选前两个信标进行跨模型对比，防止图线太多太乱
prefix = 'exper-B13-457';
kk = [2,4];

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
all_labels = {'DR','四维恒定航向偏差模型-固定信标','本发明五维模型-固定信标'};
% 调用分析函数
[radial_errors, stats] = calc_radial_error_avp(ref,all_labels, ...
    dr,all_aided_avps{:});
axis([0 8800 0 80])
xygo('时间（s）','水平位置误差（m）')
exportpngandpdf(gca, fullfile(path11, '固定信标'));
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
fig = myfigurestartup(3, 3, 'zxy');
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
        % text(xb-100, yb-100, sprintf("B%s",labels{i+1}(end)), ...
        %     'VerticalAlignment', 'top', ... % 文字位于标记下方，也可选 'top', 'middle'
        %     'HorizontalAlignment', 'center', ... % 文字水平居中
        %     'FontSize', 8, ...                  % 可选：设置字体大小
        %     'Color', color);                    % 可选：设置文字颜色与标记一致
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
legend('Location', 'north');

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
function exportpngandpdf(fig, path)
exportgraphics(fig, [path,'.png'], 'Resolution', 600);
exportgraphics(fig, [path,'.pdf'], "ContentType", "vector");
end
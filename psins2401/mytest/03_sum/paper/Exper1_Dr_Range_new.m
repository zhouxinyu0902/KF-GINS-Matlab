clear
% clc
% close all
%% 实测数据导入
for state = [4,5,7]
    clearvars -except state
    glvs
    exper = 0;
    if exper == 1
        load('data_1\deep-sea_optimized.mat');
        load('data_1\deep-sea.mat');
        avp_ref = avp_LBL_DR;
        for i = [2,4]
            BCN{4+i}=dxyz2pos([-1000,0,0],BCN{i}');
            RNG{4+i}=RCompu(avp_ref(:,7:9),BCN{4+i}) + normrnd(0,6,length(avp_ref),1);
        end
        for i=[1,3]
            BCN{4+i}=dxyz2pos([0,-1000,0],BCN{i}');
            RNG{4+i}=RCompu(avp_ref(:,7:9),BCN{4+i}) + normrnd(0,6,length(avp_ref),1);
        end
        for i=1:4
            beacon_data.fixed_pos(i,:) = BCN{i}(:);
            beacon_data.fixed_range(:,i) = RNG{i};
            beacon_data.fixed_pos(i+4,:) = BCN{4+i}(:);
            beacon_data.fixed_range(:,i+4) = RNG{4+i};
        end
        % moving_beacons.pos=[d2r(USBL_out.Transducer.Lat)',d2r(USBL_out.Transducer.Lon)',-USBL_out.Ship.Depth'];
        % moving_beacons.range = HorizRangePropaTsm(1,:);

        moving_beacons.pos=[d2r(LatLonDepTran(1,:))',d2r(LatLonDepTran(2,:))',LatLonDepTran(3,:)'];
        moving_beacons.range = HorizRangePropaTsm(1,:);
        
        % moving_beacons.range = Metrics.Ref.Horiz+normrnd(0,5,size(Metrics.Ref.Horiz));
        depth = -depther;
        depthstd = 0.4;
        dk = 0.004;

        dt = 0.5;
        ts = 0.5;
        switch state
            case 4
                x0 = [0;0;0;0];% 初始值
                dx0 = [0.002;d2r(5);5/glv.Re;5/glv.Re];% 初始值不确定性
                vk = [1e-7, d2r(0.08),0,0];
            case 5
                x0 = [0;0;0;0;0];% 初始值
                % dx0 = [0.002;d2r(4);d2r(1);5/glv.Re;5/glv.Re];% 初始值不确定性
                % vk = [1e-7, d2r(0.08), d2r(0.08), 0, 0];
                dx0 = [0.002;d2r(5);d2r(5);5/glv.Re;5/glv.Re];% 初始值不确定性
                vk = [0, d2r(0.08), d2r(0.08), 0, 0];
            case 7
                % x0 = [0;0;0;0;0;0];% 初始值
                % dx0 = [0.004;d2r(5);0.002;0.004;1/glv.Re;1/glv.Re];% 初始值不确定性
                % vk = [0, d2r(0.1), 0, 0, 0, 0];
                x0 = [0;0;0;0;0;0;0];% 初始值
                dx0 = [0.002;d2r(5);d2r(5);0.0002;0.0002;5/glv.Re;5/glv.Re];% 初始值不确定性
                vk = [1e-7, d2r(0.08),d2r(0.08), 1e-7, 1e-7, 0, 0];
        end
        rngk = 5;
        rngc = 0;
        compass(:,4) = avp_LBL_DR(:,end);
        dphi_deg_con = compass(:,3)-avp_ref(:,3);
        dphi_deg = dphi_deg_con;
    elseif exper == 0
        load('paper\data_dr_square.mat')
        % load('paper\data_dr_scan.mat')

        % 误差参数设置
        dk = 0.004;         % DVL 刻度因子误差
        dt = 0.5;          % 采样间隔 (s)

        ts = trj.ts;
        rngk = 5;
        rngc = 0;
        [beacon_data, moving_beacons,moving_beacons1] = beacon_gen_v2(rngk,avp_ref, 9, 1, 1);
        moving_beacons.pos = moving_beacons.pos(1:16:end,:);
        moving_beacons.range = moving_beacons.range(1:16:end,:);
        for i=[2,4]
            beacon_data.fixed_pos(i+4,:)=dxyz2pos([-1000,0,0],beacon_data.fixed_pos(i,:)');
            beacon_data.fixed_range(:,i+4)=RCompu(avp_ref(:,7:9),beacon_data.fixed_pos(i+4,:)) + normrnd(0,6,length(avp_ref),1);
        end
        for i=[1,3]
            beacon_data.fixed_pos(i+4,:)=dxyz2pos([0,-1000,0],beacon_data.fixed_pos(i,:)');
            beacon_data.fixed_range(:,i+4)=RCompu(avp_ref(:,7:9),beacon_data.fixed_pos(i+4,:)) + normrnd(0,6,length(avp_ref),1);
        end

        switch state
            case 4
                x0 = [0;0;0;0];% 初始值
                dx0 = [0.004;d2r(5);5/glv.Re;5/glv.Re];% 初始值不确定性
                vk = [0, d2r(0.1), 0, 0];
            case 5
                x0 = [0;0;0;0;0];% 初始值
                dx0 = [0.004;d2r(5);d2r(5);5/glv.Re;5/glv.Re];% 初始值不确定性
                vk = [0, d2r(0.1), d2r(0.1), 0, 0];
            case 7
                x0 = [0;0;0;0;0;0;0];% 初始值
                dx0 = [0.004;d2r(5);d2r(5);0.0002;0.002;5/glv.Re;5/glv.Re];% 初始值不确定性
                vk = [0, d2r(0.1),d2r(0.1), 0, 0, 0, 0];

        end

        dphi_deg_con = compass(:,3)-avp_ref(:,3);
        dphi_deg = dphi_deg_con;
        % dphi_deg = d2r(0.5)*ones(size(dphi_deg));
    elseif exper == 2
        % 针对横线和竖线轨迹进行批量分析，主要着重于可观测度
        % load('paper\data_dr_col_minus.mat')
        load('paper\data_dr_col.mat')
        % load('paper\data_dr_row.mat')
        % load('paper\data_dr_row_minus.mat')
        % 误差参数设置
        dk = 0.004;         % DVL 刻度因子误差
        dt = 0.5;          % 采样间隔 (s)

        ts = trj.ts;
        rngk = 5;
        rngc = 0;
        [beacon_data, moving_beacons,moving_beacons1] = beacon_gen_v2(rngk,avp_ref, 9, 1, 1);
        moving_beacons.pos = moving_beacons.pos(1:16:end-16,:);
        moving_beacons.range = moving_beacons.range(1:16:end-16,:);
        % 直线痕迹
        x0 = [0;0;0;0];% 初始值
        dx0 = [0.004;d2r(0.5);1/glv.Re;1/glv.Re];% 初始值不确定性
        vk = [0, d2r(0.01), 0, 0];

        % x0 = [0;0;0;0;0];% 初始值
        % dx0 = [0.004;d2r(0.5);d2r(0.5);1/glv.Re;1/glv.Re];% 初始值不确定性
        % vk = [0, d2r(0.01),d2r(0.01), 0, 0];
        dphi_deg_con = compass(:,3)-avp_ref(:,3);
        dphi_deg = dphi_deg_con;
        dphi_deg = d2r(0.5)*ones(size(dphi_deg));

        % beacon_pos_cell = {beacon_data.fixed_pos(1,:),...
        %     beacon_data.fixed_pos(2,:),...
        %     beacon_data.fixed_pos(3,:),...
        %     beacon_data.fixed_pos(4,:)};

        % labels = {'simu trajectory','beacon 1','beacon 2','beacon 3','beacon 4'};
        % fig = plot_beacon_comparison(avp_ref(:,7:9), beacon_pos_cell, labels);
        % % ylim([-10000 10000])
        % % exportgraphics(fig, fullfile('', 'D:\WPS云盘\469639050\WPS云盘\Draft\LATEX\els-cas-templates\fig\trj_vs_bea_WE.pdf'), 'Resolution', 600);
        % exportgraphics(fig, fullfile('', 'D:\WPS云盘\469639050\WPS云盘\Draft\LATEX\els-cas-templates\fig\trj_vs_bea_NS.pdf'), 'Resolution', 600);
    end
    N = length(compass);
    %%
    close all
    myfigurestartup(6,6,'prese');
    subplot 211
    plot(compass(:,4), r2d(compass(:,3)), avp_ref(:,end), r2d(avp_ref(:,3)))
    legend('罗盘','OCTANS参考系统')
    xlabel('time/s')
    ylabel('heading/deg')
    grid on
    subplot 212
    plot(compass(:,4), r2d(compass(:,3))- r2d(avp_ref(:,3)))
    grid on 
    xlabel('time/s')
    ylabel('error/deg')
    title('罗盘与参考OCTANS航向角偏差')
    

    result = analyze_compass_heading_harmonics( ...
    compass(:,4), r2d(compass(:,3)), avp_ref(:,end), r2d(avp_ref(:,3)));
        exportgraphics( ...
        result.ax1, ...
        'D:\Github\PSINS\psins2401\mytest\03_sum\figures\Compass.pdf', ...
        "ContentType", "vector");
    %%
    dr = mydr('init', avp_ref(1,7:9)', [0;0;0], ts);
    avp_dr = prealloc(N, 10);
    for i = 1:N
        t = compass(i, end);
        % --- DR 航位推算更新 ---
        dr = mydr('update', dr, depth(i), compass(i,3), vxy(i,1:2));
        avp_dr(i, :) = [dr.avp', t];
    end
    %% 1. 初始化设置
    if exper==0
        NN = size(beacon_data.fixed_pos,1)+2;
    else
        NN = size(beacon_data.fixed_pos,1)+1;
    end
    for id = 1:NN
        % for id = 1
        % for  type=["EKF","AEKF","UKF"]
        for type = "UKF"
            rng(1)
            if id == size(beacon_data.fixed_pos,1)+1
                beacon = moving_beacons.pos;
                range = moving_beacons.range;
            elseif id == size(beacon_data.fixed_pos,1)+2
                beacon = moving_beacons1.pos;
                range = moving_beacons1.range;
            else
                range  = beacon_data.fixed_range(:,id)';
                beacon = beacon_data.fixed_pos(id,:);
            end
            kf = [];
            dr = [];
            kf = myekf('init', 0.5, x0, dx0, vk, rngk);
            [avp_dr1, xk_record, pk_diag, avp_kf_out] = prealloc(N, 10, kf.m+1, 5, 10);
            ki = 1;
            dr = mydr('init', avp_ref(1,7:9)', [0;0;0], ts);
            %% 2. 组合导航主循环
            for i = 1:N
                t = compass(i, end);
                % --- DR 航位推算更新 ---
                dr = mydr('update', dr, depth(i), compass(i,3), vxy(i,1:2));
                avp_dr1(i, :) = [dr.avp', t];

                % --- EKF 预测步骤 (Time Update) ---
                kf = myekf('fk', kf, dr);
                kf = myekf('algo', kf, 'T');
                % --- EKF 量测修正 (Measurement Update) ---
                if mod(t, 8) == 0 && t~=0
                    % 确定当前信标观测值
                    if size(beacon, 1) == 1
                        dr.beacon = beacon;
                        % 考虑深度计误差，将斜距投影至水平面
                        r_meas = sqrt(range(i)^2 - (avp_ref(i,9) - dr.beacon(3))^2);
                    else
                        dr.beacon = beacon(ki+1, :);
                        r_meas = range(ki+1);
                    end

                    % 计算残差 (计算值与测量值之差)
                    kf.r_dr = sqrt(RCompu(dr.pos', dr.beacon)^2 - (depth(i) - dr.beacon(3))^2);
                    kf.yk   =   kf.r_dr - r_meas;
                    % 执行修正算法
                    kf = myekf('hk', kf, dr, 'range');
                    kf = myekf('algo', kf, 'M', type);

                    dr.pos(1:2) = dr.pos(1:2)-kf.xk(end-1:end);
                    kf.xk(end-1:end)= [0;0];
                    % kf.xk = zeros(length(x0),1);
                    % 记录滤波状态与协方差
                    xk_record(ki, :) = [kf.xk', t];
                    P = kf.Pxk(end-1:end,end-1:end);
                    pk_diag(ki, :)   = [P(:)', t];
                    Hk(ki,:) = kf.Hk(end-1:end);
                    avp_kf_out(ki, :) = [dr.avp', t]; % 记录修正时刻的AVP
                    ki = ki + 1;
                end
            end

            % 裁剪未使用的预分配空间
            xk_record(ki:end, :) = [];
            pk_diag(ki:end, :)   = [];
            Hk(ki:end, :)   = [];
            avp_kf_out(ki:end, :) = [];

            %% 3. 结果修正 (后处理)
            % 将 EKF 估计出的位置误差反馈给轨迹记录
            avp_kf_corrected = avp_kf_out;
            avp_kf_corrected(:, 7) = avp_kf_out(:, 7) - xk_record(:, end-1); % 修正纬度
            avp_kf_corrected(:, 8) = avp_kf_out(:, 8) - xk_record(:, end); % 修正经度
            avp_range{id} = avp_dr1;
            avp_range_sparse{id} = avp_kf_corrected;
            Pk{id} = pk_diag;
            HHk{id} = Hk;
            XK{id} = xk_record;

        end
    end

    %%
    if exper==1
        if length(x0)==4
            save datasaved_new/data_exper_4state.mat avp_ref avp_range avp_dr HHk beacon_data moving_beacons XK dk dphi_deg dt 
        elseif length(x0)==5
            save datasaved_new/data_exper_5state.mat avp_ref avp_range avp_dr HHk beacon_data moving_beacons XK dk dphi_deg dt
        elseif length(x0)==7
            save datasaved_new/data_exper_7state.mat avp_ref avp_range avp_dr HHk beacon_data moving_beacons XK dk dphi_deg dt 
        end
    elseif exper ==0
        if length(x0)==4
            save datasaved_new/data_simu_4state.mat avp_ref avp_range avp_dr HHk beacon_data moving_beacons XK dk dphi_deg dt moving_beacons1
        elseif length(x0)==5
            save datasaved_new/data_simu_5state.mat avp_ref avp_range avp_dr HHk beacon_data moving_beacons XK dk dphi_deg dt moving_beacons1
        elseif length(x0)==7
            save datasaved_new/data_simu_7state.mat avp_ref avp_range avp_dr HHk beacon_data moving_beacons XK dk dphi_deg dt moving_beacons1
        end
    end
    %%
    % bbb = 1:4;
    % labels = [{'DR'}, arrayfun(@(x) sprintf('Beacon %d', x), bbb, 'UniformOutput', false)];
    % % [radial_errors, stats] = calc_radial_error_avp(avp_ref,labels,avp_dr,avp_range{bbb});
    % aided_avps = cell(1, length(bbb));
    % HHks = cell(1, length(bbb));
    % beacon_pos = cell(1, length(bbb));
    % for i = 1:length(bbb)
    %     idx = bbb(i);
    %     aided_avps{i} = avp_range{idx};
    %     HHks{i} = HHk{idx};
    %     beacon_pos{i} = beacon_data.fixed_pos(idx,:);
    % end
    % [radial_errors, stats] = calc_radial_error_avp(avp_ref,labels,...
    %     avp_dr,aided_avps{:});
    % XK_1 = XK(1,bbb);
end
%%

prefix='WE';
analyze_theory_vs_actual_error_moving_beacon_v2(avp_ref, avp_dr, aided_avps, prefix, XK_1, beacon_pos, labels, dk, dphi_deg, dt)
%%
t = avp_ref(:,end);
psi = avp_ref(:,3);                  % 航向角，rad

dpsi = r2d(dphi_deg_con);            % 罗盘误差，deg
dpsi(dpsi>180)=360-dpsi(dpsi>180);
result = validate_compass_double_angle(t, psi, dpsi, ...
    'MaxOrder', 4, ...
    'UseMyFigure', true, ...
    'FigPrefix', 'zxy', ...
    'RemoveMean', false, ...
    'NumFold', 5);
%%
function fig = plot_beacon_comparison(pos_ref, beacon_pos_cell, labels)

% plot_beacon_comparison_v2: 绘制载体轨迹与信标（静止/移动）的空间分布对比图
% 输入:
%   pos_ref: 载体参考轨迹 [Lat, Lon, Alt] (单位: rad)
%   beacon_pos_cell: 包含信标位置的 cell，每个元素为 [Lat, Lon] 或 [Lat_seq, Lon_seq]
%   labels: 字符串 cell, 第一个为轨迹标签，后续为各信标标签

% --- 初始化参数 ---
Re = 6378137; % 地球长半径

colors_paper =[
    % --- 第一对：蓝色系 (比如代表算法 A) ---
    % 0.451, 0.651, 0.851; % 2. 浅灰蓝 (辅线/优化前/理论值)
    0.051, 0.251, 0.502; % 1. 深湛蓝 (主线/优化后)
    % 0.949, 0.549, 0.549  % 4. 柔和红 (辅线/优化前/理论值)
    0.651, 0.102, 0.153; % 3. 深酒红 (主线/优化后)
    0.000, 0.600, 0.498;
    1.000, 0.498, 0.055;
    0.337, 0.706, 0.914; % 2. 天蓝色
    0.902, 0.624, 0.000; % 4. 桔黄色
    % 备用色
    0.800, 0.475, 0.655; % 5. 紫红色
    0.000, 0.447, 0.698  % 6. 深蓝色
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
    else
        % 情况 B: 移动信标 (用虚线表示)
        plot(xb, yb, '--', 'LineWidth', 1.5, 'Color', color, ...
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
%%
function result = validate_compass_double_angle(t, psi, dpsi_deg, varargin)
%VALIDATE_COMPASS_DOUBLE_ANGLE Validate dominant harmonic order of compass error.
%
% 用途：
%   验证罗盘误差是否更适合用二倍角谐波模型描述。
%
% 输入：
%   t        : 时间序列，单位 s
%   psi      : 航向角，单位 rad
%   dpsi_deg : 罗盘误差，单位 deg
%
% 可选参数：
%   'MaxOrder'        : 最大测试谐波阶次，默认 4
%   'UseMyFigure'     : 是否使用 myfigurestartup，默认 true
%   'FigPrefix'       : 图名前缀，默认 'zxy'
%   'RemoveMean'      : 是否去均值后拟合，默认 false
%   'NumFold'         : 交叉验证折数，默认 5
%
% 输出：
%   result.table      : 各模型评价指标
%   result.bestOrder  : 最优单阶谐波阶次
%   result.coef       : 各模型系数
%
% 模型形式：
%   单阶 n 倍角模型：
%       dpsi = a0 + ac*cos(n*psi) + as*sin(n*psi)
%
%   混合 1+2 阶模型：
%       dpsi = a0 + a1c*cos(psi) + a1s*sin(psi)
%                  + a2c*cos(2psi) + a2s*sin(2psi)

%% -------------------- 参数解析 --------------------
p = inputParser;
addParameter(p, 'MaxOrder', 4);
addParameter(p, 'UseMyFigure', true);
addParameter(p, 'FigPrefix', 'zxy');
addParameter(p, 'RemoveMean', false);
addParameter(p, 'NumFold', 5);
parse(p, varargin{:});

maxOrder    = p.Results.MaxOrder;
useMyFigure = p.Results.UseMyFigure;
figPrefix   = p.Results.FigPrefix;
removeMean  = p.Results.RemoveMean;
numFold     = p.Results.NumFold;

%% -------------------- 数据预处理 --------------------
t        = t(:);
psi      = psi(:);
dpsi_deg = dpsi_deg(:);

valid = isfinite(t) & isfinite(psi) & isfinite(dpsi_deg);
t        = t(valid);
psi      = psi(valid);
dpsi_deg = dpsi_deg(valid);

if removeMean
    dpsi_fit = dpsi_deg - mean(dpsi_deg, 'omitnan');
else
    dpsi_fit = dpsi_deg;
end

N = length(dpsi_fit);

if N < 20
    error('有效数据点太少，无法进行可靠拟合。');
end

% 将航向角限制到 [-pi, pi]，避免数值显示混乱
psi = wrapToPi(psi);

%% -------------------- 构造候选模型并拟合 --------------------
modelNames = {};
coefCell   = {};
yhatCell   = {};
metricRows = [];

% 常值模型
X0 = ones(N, 1);
[coef0, yhat0, metric0] = fit_and_evaluate(X0, dpsi_fit);
metricRows = [metricRows; metric0];
modelNames{end+1,1} = 'Constant';
coefCell{end+1,1}   = coef0;
yhatCell{end+1,1}   = yhat0;

% 单阶谐波模型：n = 1,2,...,MaxOrder
for n = 1:maxOrder
    Xn = [ones(N,1), cos(n*psi), sin(n*psi)];
    [coefn, yhatn, metricn] = fit_and_evaluate(Xn, dpsi_fit);

    metricRows = [metricRows; metricn];
    modelNames{end+1,1} = sprintf('%d-order', n);
    coefCell{end+1,1}   = coefn;
    yhatCell{end+1,1}   = yhatn;
end

% 混合一倍角+二倍角模型，用于对照
X12 = [ones(N,1), cos(psi), sin(psi), cos(2*psi), sin(2*psi)];
[coef12, yhat12, metric12] = fit_and_evaluate(X12, dpsi_fit);

metricRows = [metricRows; metric12];
modelNames{end+1,1} = '1+2-order';
coefCell{end+1,1}   = coef12;
yhatCell{end+1,1}   = yhat12;

%% -------------------- 交叉验证误差 --------------------
cvRMSE = nan(length(modelNames), 1);

rng(1);
idx = crossvalind_local(N, numFold);

for m = 1:length(modelNames)
    err_cv = nan(N,1);

    for k = 1:numFold
        testIdx  = idx == k;
        trainIdx = ~testIdx;

        Xtrain = build_design_matrix(modelNames{m}, psi(trainIdx));
        Xtest  = build_design_matrix(modelNames{m}, psi(testIdx));

        beta = Xtrain \ dpsi_fit(trainIdx);
        err_cv(testIdx) = dpsi_fit(testIdx) - Xtest * beta;
    end

    cvRMSE(m) = sqrt(mean(err_cv.^2, 'omitnan'));
end

%% -------------------- 汇总评价表 --------------------
T = table;
T.Model      = modelNames;
T.NumParam   = metricRows(:,1);
T.RMSE_deg   = metricRows(:,2);
T.MAE_deg    = metricRows(:,3);
T.R2         = metricRows(:,4);
T.AICc       = metricRows(:,5);
T.BIC        = metricRows(:,6);
T.CV_RMSE_deg = cvRMSE;

% 单阶模型中找最优阶次，不把 1+2 混合模型纳入“单阶最优”
singleIdx = 2:(maxOrder+1);
[~, localBest] = min(T.CV_RMSE_deg(singleIdx));
bestSingleIdx = singleIdx(localBest);
bestOrder = bestSingleIdx - 1;

% 二倍角模型索引
idx2 = find(strcmp(T.Model, '2-order'), 1);
idx1 = find(strcmp(T.Model, '1-order'), 1);
idx12 = find(strcmp(T.Model, '1+2-order'), 1);

%% -------------------- 打印结果 --------------------
fprintf('\n========== Compass harmonic model validation ==========\n');
disp(T);

fprintf('\nBest single harmonic order by CV-RMSE: n = %d\n', bestOrder);

if bestOrder == 2
    fprintf('结论：在单阶谐波模型中，二倍角模型的交叉验证 RMSE 最低。\n');
else
    fprintf('注意：当前数据下，单阶最优不是二倍角，而是 %d 倍角。\n', bestOrder);
end

fprintf('\n2-order model:\n');
fprintf('  dpsi = %.6f + %.6f*cos(2psi) + %.6f*sin(2psi) deg\n', ...
    coefCell{idx2}(1), coefCell{idx2}(2), coefCell{idx2}(3));

A2 = hypot(coefCell{idx2}(2), coefCell{idx2}(3));
theta2 = atan2(coefCell{idx2}(3), coefCell{idx2}(2));
fprintf('  amplitude = %.6f deg, phase = %.6f rad\n', A2, theta2);

%% -------------------- 画图 1：时间序列拟合对比 --------------------
if useMyFigure && exist('myfigurestartup', 'file')
    myfigurestartup(3,3,figPrefix);
else
    figure;
end

plot(t, dpsi_fit, 'k', 'LineWidth', 1.0); hold on;
plot(t, yhatCell{idx1}, '--', 'LineWidth', 1.3);
plot(t, yhatCell{idx2}, '-',  'LineWidth', 1.6);

if ~isempty(idx12)
    plot(t, yhatCell{idx12}, ':', 'LineWidth', 1.6);
end

grid on;
xlabel('Time / s');
ylabel('\delta\psi / deg');
legend('Measured error', '1-order fit', '2-order fit', '1+2-order fit', ...
    'Location', 'best');
title('Compass error fitting in time domain');
xlim([min(t), max(t)]);

%% -------------------- 画图 2：航向角域拟合对比 --------------------
if useMyFigure && exist('myfigurestartup', 'file')
    myfigurestartup(3,3,figPrefix);
else
    figure;
end

psi_deg = rad2deg(wrapToPi(psi));
scatter(psi_deg, dpsi_fit, 8, 'filled'); hold on;

psi_grid = linspace(-pi, pi, 720).';
psi_grid_deg = rad2deg(psi_grid);

X1_grid  = build_design_matrix('1-order', psi_grid);
X2_grid  = build_design_matrix('2-order', psi_grid);
X12_grid = build_design_matrix('1+2-order', psi_grid);

plot(psi_grid_deg, X1_grid  * coefCell{idx1},  '--', 'LineWidth', 1.4);
plot(psi_grid_deg, X2_grid  * coefCell{idx2},  '-',  'LineWidth', 1.8);
plot(psi_grid_deg, X12_grid * coefCell{idx12}, ':',  'LineWidth', 1.8);

grid on;
xlabel('\psi / deg');
ylabel('\delta\psi / deg');
legend('Measured error', '1-order fit', '2-order fit', '1+2-order fit', ...
    'Location', 'best');
title('Compass error fitting in heading domain');
xlim([-180, 180]);

%% -------------------- 画图 3：模型指标对比 --------------------
if useMyFigure && exist('myfigurestartup', 'file')
    myfigurestartup(3,3,figPrefix);
else
    figure;
end

bar(categorical(T.Model), T.CV_RMSE_deg);
grid on;
ylabel('CV-RMSE / deg');
title('Cross-validation RMSE of different harmonic models');

%% -------------------- 画图 4：信息准则对比 --------------------
if useMyFigure && exist('myfigurestartup', 'file')
    myfigurestartup(3,3,figPrefix);
else
    figure;
end

bar(categorical(T.Model), T.BIC);
grid on;
ylabel('BIC');
title('BIC comparison of different harmonic models');

%% -------------------- 输出结果 --------------------
result = struct;
result.table     = T;
result.bestOrder = bestOrder;
result.coef      = coefCell;
result.yhat      = yhatCell;
result.psi       = psi;
result.t         = t;
result.dpsi_deg  = dpsi_fit;

end

%% ========================================================================
function [coef, yhat, metric] = fit_and_evaluate(X, y)
% 最小二乘拟合与评价指标

N = length(y);
p = size(X,2);

coef = X \ y;
yhat = X * coef;
res  = y - yhat;

SSE = sum(res.^2, 'omitnan');
MSE = SSE / N;
RMSE = sqrt(MSE);
MAE = mean(abs(res), 'omitnan');

SST = sum((y - mean(y, 'omitnan')).^2, 'omitnan');
R2 = 1 - SSE / SST;

% AIC, AICc, BIC
AIC = N * log(SSE / N) + 2 * p;

if N > p + 1
    AICc = AIC + 2*p*(p+1)/(N-p-1);
else
    AICc = nan;
end

BIC = N * log(SSE / N) + p * log(N);

metric = [p, RMSE, MAE, R2, AICc, BIC];

end

%% ========================================================================
function X = build_design_matrix(modelName, psi)
% 根据模型名称构造设计矩阵

psi = psi(:);
N = length(psi);

if strcmp(modelName, 'Constant')
    X = ones(N,1);
    return;
end

if strcmp(modelName, '1+2-order')
    X = [ones(N,1), cos(psi), sin(psi), cos(2*psi), sin(2*psi)];
    return;
end

token = regexp(modelName, '(\d+)-order', 'tokens');

if isempty(token)
    error('Unknown model name: %s', modelName);
end

n = str2double(token{1}{1});
X = [ones(N,1), cos(n*psi), sin(n*psi)];

end

%% ========================================================================
function idx = crossvalind_local(N, K)
% 不依赖 Statistics Toolbox 的 K 折索引生成

perm = randperm(N);
idx = zeros(N,1);

for i = 1:N
    idx(perm(i)) = mod(i-1, K) + 1;
end

end
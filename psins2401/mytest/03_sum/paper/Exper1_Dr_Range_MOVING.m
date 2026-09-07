clear
% clc
% close all
%% 实测数据导入
glvs
exper = 1;
if exper == 1 
    load('data_1\deep-sea_optimized.mat');
    load('data_1\deep-sea.mat');
    avp_ref = avp_LBL_DR;
    for i=[2,4]
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
    moving_beacons.pos{1}=[d2r(LatLonDepTran(1,:))',d2r(LatLonDepTran(2,:))',LatLonDepTran(3,:)'];
    moving_beacons.range{1} = HorizRangePropaTsm(1,:);

    moving_beacons.pos{2}=moving_beacons.pos{1};
    moving_beacons.range{2} = Metrics.PTSAX.Horiz_FbDela;

    moving_beacons.pos{3}=moving_beacons.pos{2};
    moving_beacons.range{3} = Metrics.PTSAG.Horiz_FbDela;

    moving_beacons.pos{4}=moving_beacons.pos{2};
    moving_beacons.range{4} = Metrics.Ref.Horiz+normrnd(0,5,size(Metrics.Ref.Horiz));
    
    depth = -depther;
    depthstd = 0.4;
    dk = 0.004;

    dt = 0.5;
    ts = 0.5;

    % x0 = [0;0;0;0];% 初始值
    % dx0 = [0.004;d2r(5);1/glv.Re;1/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1),0,0];

    % dx0 = [0.005;d2r(4);5/glv.Re;5/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1),0,0];

    % x0 = [0;0;0;0;0];% 初始值
    % dx0 = [0.004;d2r(5);d2r(5);4/glv.Re;4/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1), d2r(0.1), 0, 0];

    % x0 = [0;0;0;0];% 初始值
    % dx0 = [0.002;d2r(5);5/glv.Re;5/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.08), 0, 0];

    x0 = [0;0;0;0;0];% 初始值
    dx0 = [0.002;d2r(5);d2r(5);5/glv.Re;5/glv.Re];% 初始值不确定性
    vk = [0, d2r(0.08), d2r(0.08), 0, 0];

    % x0 = [0;0;0;0;0;0];% 初始值
    % dx0 = [0.004;d2r(5);0.002;0.004;1/glv.Re;1/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1), 0, 0, 0, 0];

    % x0 = [0;0;0;0;0;0;0];% 初始值
    % dx0 = [0.004;d2r(5);d2r(5);0.0002;0.002;1/glv.Re;1/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1),d2r(0.1), 0, 0, 0, 0];


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
    [beacon_data, ~] = beacon_gen_v2(rngk,avp_ref, 9, 1, 1);
    for i=[2,4]
        beacon_data.fixed_pos(i+4,:)=dxyz2pos([-1000,0,0],beacon_data.fixed_pos(i,:)');
        beacon_data.fixed_range(:,i+4)=RCompu(avp_ref(:,7:9),beacon_data.fixed_pos(i+4,:)) + normrnd(0,6,length(avp_ref),1);
    end
    for i=[1,3]
        beacon_data.fixed_pos(i+4,:)=dxyz2pos([0,-1000,0],beacon_data.fixed_pos(i,:)');
        beacon_data.fixed_range(:,i+4)=RCompu(avp_ref(:,7:9),beacon_data.fixed_pos(i+4,:)) + normrnd(0,6,length(avp_ref),1);
    end

    % x0 = [0;0;0;0];% 初始值
    % dx0 = [0.004;d2r(5);1/glv.Re;1/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1), 0, 0];

    % x0 = [0;0;0;0;0];% 初始值
    % dx0 = [0.004;d2r(5);d2r(5);1/glv.Re;1/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1), d2r(0.1), 0, 0];

    % x0 = [0;0;0;0;0;0];% 初始值
    % dx0 = [0.004;d2r(5);0.002;0.0002;1/glv.Re;1/glv.Re];% 初始值不确定性
    % vk = [0, d2r(0.1), 0.0002, 0.00002, 0, 0];

    dphi_deg_con = compass(:,3)-avp_ref(:,3);
    dphi_deg = dphi_deg_con;
    % dphi_deg = d2r(0.5)*ones(size(dphi_deg));
elseif exper == 2
    % 针对横线和竖线轨迹进行批量分析，主要着重于可观测度
    % load('paper\data_dr_col_minus.mat')
    % load('paper\data_dr_col.mat')
    load('paper\data_dr_row.mat')
    % load('paper\data_dr_row_minus.mat')
    % 误差参数设置
    dk = 0.004;         % DVL 刻度因子误差
    dt = 0.5;          % 采样间隔 (s)

    ts = trj.ts;
    rngk = 5;
    rngc = 0;
    [beacon_data, moving_beacons] = beacon_gen_v2(rngk,avp_ref, 9, 1, 1);
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
end

N = length(compass);
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
for id = 1:4
    % for  type=["EKF","AEKF","UKF"]
    for type = "UKF"
        rng(1)
        % if id == 1
            beacon = moving_beacons.pos{id};
            range = moving_beacons.range{id};
        % else
        %     range  = beacon_data.fixed_range(:,id)';
        %     beacon = beacon_data.fixed_pos(id,:);
        % end
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
                % r_meas = sqrt(RCompu(avp_ref(i,7:9),dr.beacon)^2-(depth(i)-dr.beacon(3))^2) + randn*6;

                % 计算残差 (计算值与测量值之差)
                kf.r_dr = sqrt(RCompu(dr.pos', dr.beacon)^2 - (depth(i) - dr.beacon(3))^2);
                kf.yk   =   kf.r_dr - r_meas;

                % 执行修正算法
                kf = myekf('hk', kf, dr, 'range');
                kf = myekf('algo', kf, 'M',type);

                dr.pos(1:2) = dr.pos(1:2)-kf.xk(end-1:end);
                kf.xk(end-1:end)= [0;0];
                % kf.xk = zeros(length(kf.xk),1);
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

save datasaved_new/data_exper_moving_4state.mat avp_ref avp_range avp_dr HHk beacon_data moving_beacons XK dk dphi_deg dt
%% 移动信标画结果图
path11 = 'D:\WPS云盘\469639050\WPS云盘\成果\1_DR_RANGE\fig\';
% close all
colors = [
        0.051, 0.251, 0.502; % 深湛蓝
        0.651, 0.102, 0.153; % 深酒红
        1.000, 0.498, 0.055;
        0.000, 0.600, 0.498;

        0.337, 0.706, 0.914; 
        0.902, 0.624, 0.000; 
        0.800, 0.475, 0.655; 
        0.000, 0.447, 0.698  
    ];
% labels={'dr','PIXOG-proposed','PTSAX','PTSAG','ref-cal'};
% ax1 = myfigurestartup(3,3,'zxy');grid on;hold on
% bbb=[2,3,1];
% % bbb=[4,1];
% for i = 1:length(bbb)
%     plot(moving_beacons.range{1,bbb(i)}-Metrics.Ref.Horiz,'-','Color',colors(i,:));
%     % plot(moving_beacons.range{1,2}-Metrics.Ref.Horiz,'.-');
%     % plot(moving_beacons.range{1,3}-Metrics.Ref.Horiz,'--');
%     % plot(moving_beacons.range{1,1}-Metrics.Ref.Horiz,'-');
% end
% legend(labels{bbb+1});
% xlabel('epoch')
% ylabel('error/m')
% xlim([0,1100])
% ylim([-20,20])
% exportgraphics(ax1, [path11,'exper-4range-cmp.pdf'], 'ContentType', 'vector');
%%
close all;
% 1. 颜色与标签设置 (沿用你的配置)
labels = {'DR', 'PIXOG-proposed', 'PTSAX', 'PTSAG', 'ref-cal'};
bbb = [2, 3, 1]; 

% 2. 创建画布
ax1 = myfigurestartup(3, 3, 'zxy'); 
grid on; hold on; box on;

% 3. 统计数据初始化
stats_results = table(); % 创建表格存储统计数据

% 4. 循环绘图与计算
for i = 1:length(bbb)
    idx = bbb(i);
    raw_err = moving_beacons.range{1, idx} - Metrics.Ref.Horiz;
    raw_err = raw_err(:); % 确保是列向量
    x = (1:length(raw_err))';
    
    % --- A. 计算统计数据 ---
    mean_err = mean(raw_err);
    std_err  = std(raw_err);
    rmse_err = sqrt(mean(raw_err.^2));
    max_err  = max(abs(raw_err));
    
    % 将结果存入表格 (方便打印)
    new_stat = table({labels{idx+1}}, mean_err, std_err, rmse_err, max_err, ...
        'VariableNames', {'Method', 'Mean', 'Std', 'RMSE', 'MaxAbs'});
    stats_results = [stats_results; new_stat];
    
    % --- B. 准备阴影图数据 ---
    % 窗口大小可以根据你的数据频率调整，例如 30
    smooth_mu = movmean(raw_err, 30);
    smooth_sigma = movstd(raw_err, 30);
    
    upper = smooth_mu + smooth_sigma;
    lower = smooth_mu - smooth_sigma;
    
    % --- C. 开始绘图 ---
    color = colors(i, :);
    
    % 绘制阴影 (不显示在图例中)
    fill([x; flipud(x)], [upper; flipud(lower)], color, ...
         'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    
    % 绘制中心主线
    lw = 1.5;
    if contains(labels{idx+1}, 'proposed'), lw = 2.5; end % 加粗 Proposed
    plot(x, smooth_mu, 'Color', color, 'LineWidth', lw, 'DisplayName', labels{idx+1});
end

% 5. 辅助参考线
line([0, 1100], [0, 0], 'Color', [0.4 0.4 0.4], 'LineStyle', '--', 'HandleVisibility', 'off');

% 6. 图形修饰
legend('Location', 'northeast');
xlabel('Epoch', 'FontName', 'Times New Roman');
ylabel('Range Error (m)', 'FontName', 'Times New Roman');
xlim([0, 1100]);
ylim([-15, 20]);

% 移除科学计数法
ax = gca;
ax.XAxis.Exponent = 0;

% 7. 打印统计数据到控制台 (直接复制到论文里)
disp('--- Range Error Statistics ---');
disp(stats_results);

% 8. 导出图像
exportpngandpdf(ax1, [path11, 'exper-4range-cmp']);
%%
bbb=[2,3,1];
label = labels([1,bbb+1]);
aided_avps = cell(1, length(bbb));
HHks = cell(1, length(bbb));
beacon_pos = cell(1, length(bbb));
aided_avps = avp_range(bbb);
HHks = HHk{bbb};
beacon_pos = moving_beacons.pos(bbb);
XK_1=XK(1,bbb);
[radial_errors, stats] = calc_radial_error_avp(avp_ref,label,...
    avp_dr,aided_avps{:});
xlim([0,8800])
exportpngandpdf(gca, [path11,'exper-moving-Radial']);
%%
bbb=[4,1];
label = labels([1,bbb+1]);
aided_avps = cell(1, length(bbb));
HHks = cell(1, length(bbb));
beacon_pos = cell(1, length(bbb));
aided_avps = avp_range(bbb);
HHks = HHk{bbb};
beacon_pos = moving_beacons.pos(bbb);
XK_1=XK(1,bbb);
[radial_errors, stats] = calc_radial_error_avp(avp_ref,label,...
    avp_dr,aided_avps{:});
xlim([0,8800])
exportpngandpdf(gca, [path11,'exper-moving-Radial-2']);
%%
trjsee(avp_LBL_DR,'2d',avp_dr,avp_range{1})
legend('truth','DR','PIXOG-proposed')
ylim([-1000,300])
axis equal
exportgraphics(gca, [path11,'trjcmp3d.png'], 'Resolution', 600,'ContentType', 'vector');
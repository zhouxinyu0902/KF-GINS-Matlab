function Metrics = eval_acoustic_ranges(USBL_out, avp_ref, tt_lbl, tt_usbl, isfig)
% EVAL_ACOUSTIC_RANGES 评估声学定位距离，并进行传播时间滞后深度补偿
%
% 输入:
%   USBL_out : 阶段4输出的 USBL 数据结构体 (包含换能器位置、USBL各观测分量)
%   avp_ref  : 作为基准的参考轨迹 (通常是 LBL 或 LBL+DR 融合结果)
%   tt_lbl   : 高频参考轨迹的时间轴
%   tt_usbl  : USBL 低频观测时间轴
%   isfig    : 是否绘制误差对比图 (1/0)
%
% 输出:
%   Metrics  : 包含基准距离、各分量(PTSAG, PTSAX, PIXOG)误差量化结果的结构体

    if nargin < 5, isfig = 0; end
    fprintf('开始评估声学距离与深度反馈补偿 (Phase 6)...\n');

    %% 1. 建立参考真值 (Reference Ground Truth)
    % 提取参考轨迹的经纬度深度
    Lat_ref = rad2deg(avp_ref(:, 7));
    Lon_ref = rad2deg(avp_ref(:, 8));
    Depth_ref = avp_ref(:, 9); % 假设 avp 中 Down 为正，转为负值深度
    
    % 将参考轨迹插值对齐到 USBL 的时间轴上
    Lat_ref_sync   = interp1(tt_lbl, Lat_ref, tt_usbl, 'linear', 'extrap');
    Lon_ref_sync   = interp1(tt_lbl, Lon_ref, tt_usbl, 'linear', 'extrap');
    Depth_ref_sync = interp1(tt_lbl, Depth_ref, tt_usbl, 'linear', 'extrap');
    
    % 转为 UTM
    [X_ref, Y_ref] = ll2utm(Lat_ref_sync, Lon_ref_sync);
    
    % 换能器 (Transducer) 与母船深度
    X_tran = USBL_out.Transducer.XUTM;
    Y_tran = USBL_out.Transducer.YUTM;
    Z_ship = -USBL_out.Ship.Depth; % 母船深度(负值)

    % 计算参考斜距与水平距离 (使用内置子函数)
    [Metrics.Ref.Slant, Metrics.Ref.Horiz] = calc_range(X_tran, Y_tran, Z_ship, X_ref, Y_ref, Depth_ref_sync);

    %% 2. 核心：处理传播时间导致的历史深度滞后
    % USBL_out.TimeH1 是单程传播时间。声波发射瞬间，AUV 的真实时间其实是当前时刻减去传播时间
    t_delayed = tt_usbl - USBL_out.TimeH1 - 0.02; % 减去传播时间及固定延迟
    
    % 在高频时间轴 (tt_lbl) 上精准插值，获取声波发射瞬间的“真实历史深度”
    Depth_delayed = interp1(tt_lbl, Depth_ref, t_delayed, 'linear', 'extrap');
    
    % 记录当前深度和滞后深度，供各模块 Feedback 修正使用
    Z_cabin_current = Depth_ref_sync;
    Z_cabin_delayed = Depth_delayed;

    %% 3. PTSAG (绝对定位) 距离计算与误差评估
    [X_ptsag, Y_ptsag] = ll2utm(USBL_out.LatHov, USBL_out.LonHov);
    Z_ptsag = -USBL_out.DepthHov;

    % 原始计算
    [Slant_ptsag, Horiz_ptsag] = calc_range(X_tran, Y_tran, Z_ship, X_ptsag, Y_ptsag, Z_ptsag);
    
    % 深度反馈修正 (Feedback): 维持斜距不变，用更高精度的舱内深度反算水平距离
    Horiz_ptsag_fb_curr = sqrt(max(0, Slant_ptsag.^2 - (Z_ship - Z_cabin_current).^2));
    Horiz_ptsag_fb_dela = sqrt(max(0, Slant_ptsag.^2 - (Z_ship - Z_cabin_delayed).^2)); % 滞后深度修正

    % 误差记录 (减去 Reference)
    Metrics.PTSAG.ErrSlant = Slant_ptsag - Metrics.Ref.Slant;
    Metrics.PTSAG.ErrHoriz_Raw = Horiz_ptsag - Metrics.Ref.Horiz;
    Metrics.PTSAG.ErrHoriz_FbCurr = Horiz_ptsag_fb_curr - Metrics.Ref.Horiz;
    Metrics.PTSAG.ErrHoriz_FbDela = Horiz_ptsag_fb_dela - Metrics.Ref.Horiz;
    Metrics.PTSAG.Slant = Slant_ptsag;
    Metrics.PTSAG.Horiz_FbCurr = Horiz_ptsag_fb_curr;
    Metrics.PTSAG.Horiz_Raw = Horiz_ptsag;
    Metrics.PTSAG.Horiz_FbDela = Horiz_ptsag_fb_dela;
    %% 4. PTSAX (相对位移) 距离计算与误差评估
    % PTSAX 直接给出了相对 X 和 Y 的位移，因此它的水平距离就是 sqrt(X^2 + Y^2)
    Horiz_ptsax = sqrt(USBL_out.XForward.^2 + USBL_out.YStarboard.^2);
    Slant_ptsax = sqrt(Horiz_ptsax.^2 + (USBL_out.DepthHovPTSAX - (-Z_ship)).^2);
    
    % 深度反馈修正
    Horiz_ptsax_fb_curr = sqrt(max(0, Slant_ptsax.^2 - (Z_ship - Z_cabin_current).^2));
    Horiz_ptsax_fb_dela = sqrt(max(0, Slant_ptsax.^2 - (Z_ship - Z_cabin_delayed).^2));
    
    % 误差记录
    Metrics.PTSAX.ErrSlant = Slant_ptsax - Metrics.Ref.Slant;
    Metrics.PTSAX.ErrHoriz_Raw = Horiz_ptsax - Metrics.Ref.Horiz;
    Metrics.PTSAX.ErrHoriz_FbCurr = Horiz_ptsax_fb_curr - Metrics.Ref.Horiz;
    Metrics.PTSAX.ErrHoriz_FbDela = Horiz_ptsax_fb_dela - Metrics.Ref.Horiz;
    Metrics.PTSAX.Slant = Slant_ptsax;
    Metrics.PTSAX.Horiz_Raw = Horiz_ptsax;
    Metrics.PTSAX.Horiz_FbCurr = Horiz_ptsax_fb_curr;
    Metrics.PTSAX.Horiz_FbDela = Horiz_ptsax_fb_dela;
    %% 5. PIXOG (传播时间模型) 距离计算与误差评估
    % 利用传播时间反推斜距: Slant = Time * SoundSpeed (此处假定已经通过你的 PropaTcmp 处理为 Est_range)
    % 注: 这里简化调用了外部变量，如果你有一个函数 calc_propa_range，请在此替换。
    % 这里我们用直接反推逻辑演示：
    % 假设通过时间模型计算出了一个最优水平距离 Est_Horiz_sm
    % (由于你原文调用了 PropaTcmp，你需要确保该函数可用。如果只是要验证深度补偿，可以直接对比 PTSAG/PTSAX)
    % 强制展平为行向量，防止维度扩展导致内存溢出
    Est_range = PropaTcmp(-Z_ship(:)', -Z_cabin_delayed(:)', USBL_out.TimeH1(:)');
    
    % --- 补充：PIXOG 误差计算与平滑 ---
    % 对估算的水平距离进行平滑 (对应你原代码的 Proposed 和 ML 算法)
    Est1_sm = smooth(tt_usbl, Est_range(1,:), 0.008, 'rloess')';
    Est2_sm = smooth(tt_usbl, Est_range(2,:), 0.008, 'rloess')';
    
    % 记录水平误差
    Metrics.PIXOG.ErrHoriz_Sm1 = Est1_sm - Metrics.Ref.Horiz;
    Metrics.PIXOG.ErrHoriz_Sm2 = Est2_sm - Metrics.Ref.Horiz;
    
    % 反推 PIXOG 斜距并记录误差 (用于最后一张综合对比图)
    dhgt = Z_ship(:)' - Z_cabin_delayed(:)'; 
    Slant_propa1 = sqrt(Est1_sm.^2 + dhgt.^2);
    Metrics.PIXOG.ErrSlant1 = Slant_propa1 - Metrics.Ref.Slant;
    
    % 保存进结构体
    Metrics.PIXOG.Est_range = Est_range;
    Metrics.PIXOG.Est_range_sm = [Est1_sm; Est2_sm];

    fprintf('距离量化与补偿评估完成！\n\n');

    %% 6. 可视化绘图 (如果 isfig == 1)
    if isfig
        myfigurestartup(7, 5, 'paper'); % 稍微加高一点画布，适应 2x2 布局
        
        % 1. PTSAG 水平误差对比图
        subplot(2, 2, 1);
        plot(tt_usbl, Metrics.PTSAG.ErrHoriz_Raw, 'g--', 'DisplayName', 'PTSAG Raw'); hold on;
        plot(tt_usbl, Metrics.PTSAG.ErrHoriz_FbCurr, 'b.', 'DisplayName', 'Fb (Current Z)');
        plot(tt_usbl, Metrics.PTSAG.ErrHoriz_FbDela, 'r.', 'DisplayName', 'Fb (Delayed Z)');
        yline(0, 'k-', 'LineWidth', 1); grid on; xlim([0, tt_usbl(end)]); ylim([-10, 10]);
        xygo('Time (s)', 'Horiz Error (m)'); title('PTSAG Horizontal Range Errors'); legend('Location', 'best');
        
        % 2. PTSAX 水平误差对比图
        subplot(2, 2, 2);
        plot(tt_usbl, Metrics.PTSAX.ErrHoriz_Raw, 'g--', 'DisplayName', 'PTSAX Raw'); hold on;
        plot(tt_usbl, Metrics.PTSAX.ErrHoriz_FbCurr, 'b.', 'DisplayName', 'Fb (Current Z)');
        plot(tt_usbl, Metrics.PTSAX.ErrHoriz_FbDela, 'r.', 'DisplayName', 'Fb (Delayed Z)');
        yline(0, 'k-', 'LineWidth', 1); grid on; xlim([0, tt_usbl(end)]); ylim([-10, 10]);
        xygo('Time (s)', 'Horiz Error (m)'); title('PTSAX Horizontal Range Errors'); legend('Location', 'best');
        
        % 3. PIXOG (传播时间推算) 水平误差对比图
        subplot(2, 2, 3);
        plot(tt_usbl, Metrics.PIXOG.ErrHoriz_Sm1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Proposed-sm'); hold on;
        plot(tt_usbl, Metrics.PIXOG.ErrHoriz_Sm2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'ML-sm');
        yline(0, 'k-', 'LineWidth', 1); grid on; xlim([0, tt_usbl(end)]); ylim([-10, 10]);
        xygo('Time (s)', 'Horiz Error (m)'); title('PIXOG Horizontal Range Errors'); legend('Location', 'best');

        % 4. 综合斜距误差对比图 (汇集三种数据源)
        subplot(2, 2, 4);
        plot(tt_usbl, Metrics.PTSAG.ErrSlant, 'm.', 'DisplayName', 'PTSAG Slant Error'); hold on;
        plot(tt_usbl, Metrics.PTSAX.ErrSlant, 'c.', 'DisplayName', 'PTSAX Slant Error');
        plot(tt_usbl, Metrics.PIXOG.ErrSlant1, 'k-', 'LineWidth', 1, 'DisplayName', 'PIXOG Slant Error');
        yline(0, 'k-', 'LineWidth', 1); grid on; xlim([0, tt_usbl(end)]); ylim([-15, 15]);
        xygo('Time (s)', 'Slant Range Error (m)'); title('Slant Range Consistency'); legend('Location', 'best');
    end
end

%% ================== 局部辅助函数 ==================
function [slant, horiz] = calc_range(x1, y1, z1, x2, y2, z2)
% 计算两点之间的斜距与水平距离
    horiz = sqrt((x1 - x2).^2 + (y1 - y2).^2);
    slant = sqrt(horiz.^2 + (z1 - z2).^2);
end
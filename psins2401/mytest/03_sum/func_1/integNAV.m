function [avp_LBL_DR,avp_usbl_raw,avp_lbl_raw]=integNAV(USBL_out,LBL_out,cfg,depther,octans,vxy,compass,tt_lbl)
avp_lbl_raw  = LBL_out.avp_m;
avp_usbl_raw = [zeros(length(cfg.tt_usbl), 6), d2r([USBL_out.LatHov', USBL_out.LonHov']),...
    -depther(1:16:16*length(cfg.tt_usbl)), cfg.tt_usbl'];
% 选择最高精度的 LBL+OCTANS+DVL 融合轨迹作为 Reference
[avp_LBL_DR, avp_DR_OCTANS] = AcousticDeadR('LBL', tt_lbl, avp_lbl_raw, d2r(0.2), avp_lbl_raw, octans, vxy, depther, 0);
[avp_USBL_DR, ~] = AcousticDeadR('USBL', tt_lbl, avp_lbl_raw, d2r(0.2), avp_usbl_raw, octans, vxy, depther, 0);
[avp_LBL_DR_compass, avp_DR_compass] = AcousticDeadR('LBL', tt_lbl, avp_lbl_raw, d2r(0.3), avp_lbl_raw, compass, vxy, depther, 0);
% [avp_USBL_DR_compass, ~] = AcousticDeadR('USBL', tt_lbl, avp_lbl_raw, d2r(0.2), avp_usbl_raw, compass, vxy, depther, 1);
pos_ref = avp_LBL_DR(:,[7,8,10]);
usbl_result = avp_usbl_raw(:,[7,8,10]);
pos_lbl = avp_lbl_raw(:,[7,8,10]);
pos_dr2 = avp_DR_compass(:,[7,8,10]);
pos_dr = avp_DR_OCTANS(:,[7,8,10]);
pos_integ = avp_USBL_DR(:,[7,8,10]);
pos_integ2 = avp_LBL_DR_compass(:,[7,8,10]);

plot_nav_analysis(pos_ref,{'参考(lbl-octans组合导航)','usbl声学定位','lbl声学定位','compass_dr', ...
    'octans_dr','usbl-octans组合导航','lbl-compass组合导航'} , ...
    usbl_result, pos_lbl,pos_dr2, pos_dr, pos_integ, pos_integ2);
end
function stats = plot_nav_analysis(pos_ref, labels, varargin)
% PLOT_NAV_ANALYSIS 综合导航评估函数 (基于时间戳自动对齐)
%
% 输入参数:
%   pos_ref  : N*3 矩阵，[纬度(rad), 经度(rad), 时间(s)] (连续高频参考真值)
%   labels   : 元胞数组，包含各个输入矩阵的图例名称，如 {'LBL Reference', 'USBL Cleaned', 'Fused DR'}
%   varargin : 多个 M*3 矩阵，[纬度(rad), 经度(rad), 时间(s)] (待评估数据)
%
% 示例:
%   stats = plot_nav_analysis(pos_lbl, {'LBL', 'USBL', 'DR'}, pos_usbl, pos_dr);

    num_targets = length(varargin);
    if num_targets == 0
        error('未提供待比较的位置矩阵。');
    end
    stats = cell(1, num_targets); % 初始化统计 cell
    
    % 地球参数
    Re = 6378137.0; 
    
    % --- 处理参考真值 (Reference) ---
    % 以参考轨迹的第一个点作为局部坐标系原点 (0,0) 米
    lat0 = pos_ref(1, 1);
    lon0 = pos_ref(1, 2);
    t_ref = pos_ref(:, end); % 最后一列为时间
    
    % 将参考轨迹转换为局部东北向 (米)
    N_ref_m = (pos_ref(:, 1) - lat0) .* Re;
    E_ref_m = (pos_ref(:, 2) - lon0) .* Re .* cos(lat0);
    
    % --- 初始化图形窗口 ---
    fig_traj  = myfigurestartup(5,5,'prese');
    fig_comp  = myfigurestartup(12,5,'prese');
    fig_error = myfigurestartup(12,7,'prese');
    colors = lines(num_targets);
    
    % --- 1. 绘制参考真值的二维平面轨迹 ---
    figure(fig_traj); hold on; grid on; axis equal;
    plot(E_ref_m, N_ref_m, 'k-', 'LineWidth', 1.5, 'DisplayName', labels{1});
    xlabel('East (m)'); ylabel('North (m)');
    title('水平定位轨迹对比 (局部 ENU 坐标系)');
    
    % --- 2. 绘制参考真值的绝对位置分量 ---
    figure(fig_comp);
    sub_N = subplot(1, 2, 1); hold on; grid on;
    plot(t_ref, N_ref_m, 'k-', 'LineWidth', 1.5, 'DisplayName', [labels{1} ' North']);
    ylabel('North (m)'); xlabel('Time (s)'); title('北向分量时间序列');
    
    sub_E = subplot(1, 2, 2); hold on; grid on;
    plot(t_ref, E_ref_m, 'k-', 'LineWidth', 1.5, 'DisplayName', [labels{1} ' East']);
    ylabel('East (m)'); xlabel('Time (s)'); title('东向分量时间序列');
    
    % 打印表头
    fprintf('\n=================================================================================\n');
    fprintf('                               导航误差量化分析报表\n');
    fprintf('=================================================================================\n');
    fprintf('| 数据源名称     | 东向RMSE | 北向RMSE | 水平RMSE | 东向均值 | 北向均值 | 最大水平误差 |\n');
    fprintf('---------------------------------------------------------------------------------\n');
    
    % --- 遍历所有的目标数据进行分析 ---
    for i = 1:num_targets
        pos_i = varargin{i};
        t_i = pos_i(:, end); % 获取目标数据的时间戳
        
        % 转为局部坐标系 (米)
        N_i_m = (pos_i(:, 1) - lat0) .* Re;
        E_i_m = (pos_i(:, 2) - lon0) .* Re .* cos(lat0);
        
        % === 核心改进：基于时间戳的插值对齐 ===
        % 无论采样率如何，利用 t_i 在参考轨迹上插值，得到严格时间对齐的基准位置
        N_ref_interp = interp1(t_ref, N_ref_m, t_i, 'linear', 'extrap');
        E_ref_interp = interp1(t_ref, E_ref_m, t_i, 'linear', 'extrap');
        
        % 计算时间对齐后的误差 (米)
        err_N = N_i_m - N_ref_interp;
        err_E = E_i_m - E_ref_interp;
        err_R = sqrt(err_N.^2 + err_E.^2);
        
        % 统计数据记录
        stats{1,i}.name = labels{i+1};
        stats{1,i}.mean_E = mean(err_E, 'omitnan');
        stats{1,i}.mean_N = mean(err_N, 'omitnan');
        stats{1,i}.rmse_E = sqrt(mean(err_E.^2, 'omitnan'));
        stats{1,i}.rmse_N = sqrt(mean(err_N.^2, 'omitnan'));
        stats{1,i}.rmse_2D = sqrt(mean(err_R.^2, 'omitnan'));
        stats{1,i}.max_2D = max(err_R, [], 'omitnan');
        
        % 打印表格
        fprintf('| %-14s | %8.3f | %8.3f | %8.3f | %8.3f | %8.3f | %12.3f |\n', ...
            stats{1,i}.name, stats{1,i}.rmse_E, stats{1,i}.rmse_N, stats{1,i}.rmse_2D, ...
            stats{1,i}.mean_E, stats{1,i}.mean_N, stats{1,i}.max_2D);
            
        % 根据数据特性选择线型：名字包含 USBL 则用散点，否则用实线
        if contains(upper(stats{1,i}.name), 'USBL')
            line_style = '.'; marker_size = 10;
        else
            line_style = '-'; marker_size = 6; 
        end
        
        % --- 图1：叠加轨迹 ---
        figure(fig_traj);
        plot(E_i_m, N_i_m, line_style, 'Color', colors(i,:), 'MarkerSize', marker_size, 'DisplayName', stats{1,i}.name);
        
        % --- 图2：叠加分量 ---
        figure(fig_comp);
        subplot(sub_N); plot(t_i, N_i_m, line_style, 'Color', colors(i,:), 'MarkerSize', marker_size, 'DisplayName', [stats{1,i}.name ' North']);
        subplot(sub_E); plot(t_i, E_i_m, line_style, 'Color', colors(i,:), 'MarkerSize', marker_size, 'DisplayName', [stats{1,i}.name ' East']);
        
        % --- 3. 绘制误差曲线 (横轴使用各自独立的时间戳 t_i) ---
        figure(fig_error);
        subplot(3,1,1); hold on; grid on;
        plot(t_i, err_E, 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', sprintf('%s (RMSE: %.1fm)', stats{1,i}.name, stats{1,i}.rmse_E));
        ylabel('East Error (m)'); title('位置误差时间序列');
        
        subplot(3,1,2); hold on; grid on;
        plot(t_i, err_N, 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', sprintf('%s (RMSE: %.1fm)', stats{1,i}.name, stats{1,i}.rmse_N));
        ylabel('North Error (m)');
        
        subplot(3,1,3); hold on; grid on;
        plot(t_i, err_R, 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', sprintf('%s (2D RMSE: %.1fm)', stats{1,i}.name, stats{1,i}.rmse_2D));
        ylabel('Horizontal Error (m)'); xlabel('Time (s)');
    end
    fprintf('=================================================================================\n\n');
    
    % 添加图例
    figure(fig_traj); legend('Location', 'best', 'Interpreter', 'none');
    figure(fig_comp); subplot(sub_N); legend('Location', 'best', 'Interpreter', 'none'); subplot(sub_E); legend('Location', 'best', 'Interpreter', 'none');
    figure(fig_error); subplot(3,1,1); legend('Location', 'best', 'Interpreter', 'none'); subplot(3,1,2); legend('Location', 'best', 'Interpreter', 'none'); subplot(3,1,3); legend('Location', 'best', 'Interpreter', 'none');
end

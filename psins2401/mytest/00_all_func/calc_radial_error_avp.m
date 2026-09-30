function [radial_errors, stats] = calc_radial_error_avp(avp_ref,labels, varargin)
    glvs; 
    num_tracks = length(varargin);
    radial_errors = cell(num_tracks, 1);
    
    % --- 动态配色系统 ---
    % 使用 lines 颜色图，并加深参考轨迹颜色
    color_map = [
    0.051, 0.251, 0.502;   % 1  深湛蓝
    0.651, 0.102, 0.153;   % 2  深酒红
    0.000, 0.600, 0.498;   % 3  青绿色
    1.000, 0.498, 0.055;   % 4  橙色
    0.337, 0.706, 0.914;   % 5  浅蓝
    0.902, 0.624, 0.000;   % 6  金黄色
    0.800, 0.475, 0.655;   % 7  紫红色
    0.000, 0.447, 0.698;   % 8  标准蓝
    0.350, 0.700, 0.250;   % 9  柔和绿

    0.580, 0.404, 0.741;   % 10 紫色
    0.835, 0.369, 0.000;   % 11 深橙
    0.000, 0.620, 0.780;   % 12 蓝绿色
    0.550, 0.337, 0.294;   % 13 棕色
    0.890, 0.467, 0.761;   % 14 粉紫色
    0.450, 0.450, 0.450;   % 15 中性灰
];
    line = {'--',':','-','--',':','-','--',':','-'};
    ref_color = [0.15 0.15 0.15]; % 深灰色作为参考基准
    
    t_ref = avp_ref(:, 10);
    pos_ref = avp_ref(:, 7:9);
    
    % 调用自定义画布，若无则默认

    myfigurestartup(3, 3, 'zxy');


    %% --- 左侧：误差曲线绘制 ---
    hold on; grid on; box on;
    % labels = {};
    
    for i = 1:num_tracks
        curr_avp = varargin{i};
        t_curr = curr_avp(:, 10);
        pos_curr = curr_avp(:, 7:9);
        
        % 1. 时间对齐逻辑 (线性插值)
        t_start = max(t_ref(1), t_curr(1));
        t_end = min(t_ref(end), t_curr(end));
        valid_idx = (t_ref >= t_start & t_ref <= t_end);
        t_eval = t_ref(valid_idx);
        
        if isempty(t_eval), continue; end
        
        % 2. 坐标插值 (处理经度跳变)
        pos_interp = zeros(length(t_eval), 3);
        pos_interp(:, 1) = interp1(t_curr, pos_curr(:, 1), t_eval, 'linear');
        pos_interp(:, 2) = interp1(t_curr, unwrap(pos_curr(:, 2)), t_eval, 'linear');
        pos_interp(:, 3) = interp1(t_curr, pos_curr(:, 3), t_eval, 'linear');
        
        % 3. 计算三维径向误差
        % RCompu 计算的是地表两点间的物理距离(m)
        err = RCompu(pos_ref(valid_idx, :), pos_interp);
        radial_errors{i} = [t_eval, err];
        
        % 4. 统计分析
        stats(i).max  = max(err);
        stats(i).rms  = sqrt(mean(err.^2));
        stats(i).mean = mean(err);
        stats(i).std  = std(err);
        
        % 5. 绘图
        c = color_map(i, :);
        plot(t_eval, err, 'Color', c,'LineStyle',line{i});
        % 绘制 RMS 水平虚线 (视觉辅助)
        yline(stats(i).rms, '--', 'Color', c, 'Alpha', 0.5, 'HandleVisibility', 'off','DisplayName', labels{i});
        
        % labels{i} = sprintf('Track %d (RMS: %.2f)', i-1, stats(i).rms);
    end
    
    xlabel('Time / s'); ylabel('Radial Error / m');
    % title('Radial Position Error');
    legend(labels, 'Location', 'best');

    % set(gca, 'DefaultTextFontName', 'TimesSimSun'); % 文本标注：TimesSimSun
    % set(gca, 'DefaultAxesFontName', 'TimesSimSun'); % 坐标轴：TimesSimSun
    % set(gca, 'DefaultLegendFontName', 'TimesSimSun'); % 图例：TimesSimSun
    % %% --- 右侧：2D 轨迹可视化 ---
    % subplot(1, 2, 2);
    % % 确保 trjsee 使用同样的配色逻辑
    % trjsee(avp_ref, '2d', varargin{:});
    % title('Trajectory Comparison (2D)');
    
    %% --- 控制台输出报告 ---
    fprintf('\n%s [误差分析报告] %s\n', repmat('=', 1, 10), repmat('=', 1, 10));
    fprintf('%-8s | %-8s | %-8s | %-8s | %-8s\n', 'ID', 'Max(m)', 'RMS(m)', 'Mean(m)', 'STD(m)');
    fprintf('%s\n', repmat('-', 1, 55));
    for i = 1:num_tracks
        if isempty(stats(i).max), continue; end
        fprintf('%s | %-8.2f | %-8.2f | %-8.2f | %-8.2f\n', ...
            labels{i}, stats(i).max, stats(i).rms, stats(i).mean, stats(i).std);
    end
    fprintf('%s\n', repmat('=', 1, 35));
end
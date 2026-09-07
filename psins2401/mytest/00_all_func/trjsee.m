function trjsee(avp, type, varargin)
% TRJSEE 轨迹对比可视化工具
%
% 用法：
%   trjsee(avp_true, '2d', avp1, avp2, ...)
%   trjsee(avp_true, '3d', avp1, avp2, ...)
%
% avp(:,7:9) 分别为 lat, lon, hgt，单位通常为 rad, rad, m

    %% ---------- 基础检查 ----------
    if nargin < 2
        error('用法错误：trjsee(avp, type, varargin)，type 需要指定为 ''2d'' 或 ''3d''。');
    end

    if ~isnumeric(avp) || size(avp, 2) < 9
        error('avp 必须是数值矩阵，且至少包含 9 列，其中第 7-9 列为位置。');
    end

    n = numel(varargin);

    for i = 1:n
        curr_avp = varargin{i};
        if ~isnumeric(curr_avp) || size(curr_avp, 2) < 9
            error('第 %d 条对比轨迹格式错误：轨迹矩阵至少需要包含 9 列。', i);
        end
    end

    %% ---------- 配色 ----------
    ref_color = [0.15 0.15 0.15];      % 参考轨迹：深灰色
    line_colors = getLineColors(n);    % 对比轨迹颜色

    labels = cell(1, n + 1);
    labels{1} = 'Reference (True)';
    for i = 1:n
        labels{i + 1} = sprintf('Track %d', i);
    end

    %% ---------- 新建图窗 ----------
    if exist('myfigurestartup', 'file') == 2
        myfigurestartup(3, 3, 'zxy');
    else
        figure;
    end

    hold on;
    grid on;
    box on;

    h = gobjects(n + 1, 1);

    %% ---------- 绘图 ----------
    switch lower(type)

        case '2d'
            % 所有轨迹统一以参考轨迹起点作为局部坐标原点
            origin = avp(1, 7:9)';

            dxyz_ref = pos2dxyz(avp(:, 7:9), origin);
            h(1) = plot(dxyz_ref(:, 1), dxyz_ref(:, 2), ...
                'Color', ref_color, ...
                'LineWidth', 2.5);

            for i = 1:n
                curr_avp = varargin{i};
                dxyz_curr = pos2dxyz(curr_avp(:, 7:9), origin);

                h(i + 1) = plot(dxyz_curr(:, 1), dxyz_curr(:, 2), ...
                    'Color', line_colors(i, :), ...
                    'LineWidth', 1.3);
            end

            % 起点标记
            plot(0, 0, 'p', ...
                'MarkerSize', 11, ...
                'MarkerFaceColor', 'y', ...
                'MarkerEdgeColor', 'r', ...
                'HandleVisibility', 'off');

            if exist('xygo', 'file') == 2
                xygo('East / m', 'North / m');
            else
                xlabel('East / m');
                ylabel('North / m');
            end

            axis equal;
            title('2D Trajectory Comparison');

        case '3d'
            h(1) = plot3(r2d(avp(:, 8)), r2d(avp(:, 7)), avp(:, 9), ...
                'Color', ref_color, ...
                'LineWidth', 2.5);

            for i = 1:n
                curr_avp = varargin{i};

                h(i + 1) = plot3(r2d(curr_avp(:, 8)), ...
                                  r2d(curr_avp(:, 7)), ...
                                  curr_avp(:, 9), ...
                    'Color', line_colors(i, :), ...
                    'LineWidth', 1.3);
            end

            % 起点标记
            plot3(r2d(avp(1, 8)), r2d(avp(1, 7)), avp(1, 9), 'p', ...
                'MarkerSize', 12, ...
                'MarkerFaceColor', 'y', ...
                'MarkerEdgeColor', 'r', ...
                'HandleVisibility', 'off');

            xlabel('Lon / deg');
            ylabel('Lat / deg');
            zlabel('Hgt / m');

            view(3);
            axis tight;
            title('3D Trajectory Comparison');

        otherwise
            error('type 参数错误：只能是 ''2d'' 或 ''3d''。');
    end

    %% ---------- 图例与显示优化 ----------
    legend(h, labels, ...
        'Location', 'best', ...
        'Interpreter', 'none');

    set(gca, ...
        'FontName', 'Times New Roman', ...
        'FontSize', 11, ...
        'LineWidth', 1);

end


%% ============================================================
function colors = getLineColors(n)
%GETLINECOLORS 获取对比轨迹颜色
% 解决 n > 8 时颜色越界的问题

    if n == 0
        colors = zeros(0, 3);
        return;
    end

    if n == 1
        colors = [0.651, 0.102, 0.153];   % 单条对比轨迹默认红色
        return;
    end

    base_colors = [
        0.051, 0.251, 0.502;   % 深蓝
        0.651, 0.102, 0.153;   % 深红
        0.000, 0.600, 0.498;   % 青绿
        1.000, 0.498, 0.055;   % 橙色
        0.337, 0.706, 0.914;   % 浅蓝
        0.902, 0.624, 0.000;   % 金黄
        0.800, 0.475, 0.655;   % 紫粉
        0.000, 0.447, 0.698    % 蓝色
    ];

    m = size(base_colors, 1);

    if n <= m
        colors = base_colors(1:n, :);
    else
        % 超过基础颜色数量时，循环使用颜色，避免索引越界
        colors = repmat(base_colors, ceil(n / m), 1);
        colors = colors(1:n, :);
    end
end
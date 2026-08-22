function fig = plot_navigation_scene(trajectory, varargin)
% PLOT_NAVIGATION_SCENE 轨迹与信标对比可视化函数
% 
% 语法:
%   plot_navigation_scene(trajectory, 'static', S, 'moving', M, 'type', 'xyz')
%
% 输入参数:
%   trajectory: N*3 矩阵 [x, y, z] 或 [lat, lon, alt]
%   'static'  : (可选) M*3 矩阵，每行为一个静止信标坐标
%   'moving'  : (可选) N*3 矩阵，随时间变化的移动信标坐标
%   'type'    : (可选) 'xyz' (默认) 或 'lla'
%   'unit'    : (可选) 'm' 或 'km' (仅对xyz有效，默认km)

    %% 1. 解析输入参数
    p = inputParser;
    addRequired(p, 'trajectory');
    addParameter(p, 'static', []);
    addParameter(p, 'moving', []);
    addParameter(p, 'type', 'xyz', @(x) any(validatestring(x, {'xyz', 'lla'})));
    addParameter(p, 'unit', 'km');
    parse(p, trajectory, varargin{:});
    
    trj = p.Results.trajectory;
    stc = p.Results.static;
    mov = p.Results.moving;
    mode = lower(p.Results.type);
    unit_scale = 1;
    if strcmpi(p.Results.unit, 'km') && strcmpi(mode, 'xyz')
        unit_scale = 1000;
    end

    %% 2. 创建绘图
    fig = myfigurestartup(3,3,'paper');
    
    hold on; grid on; box on;
    
    % 绘制轨迹
    plot3(trj(:,1)/unit_scale, trj(:,2)/unit_scale, trj(:,3)/unit_scale, ...
          'b-', 'LineWidth', 2, 'DisplayName', '载体轨迹');
    
    % 绘制静止信标 (如果是多个，循环绘制)
    if ~isempty(stc)
        plot3(stc(:,1)/unit_scale, stc(:,2)/unit_scale, stc(:,3)/unit_scale, ...
              'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
              'DisplayName', '静止信标');
        % 为每个静止信标添加编号标注
        for i = 1:size(stc, 1)
            text(stc(i,1)/unit_scale, stc(i,2)/unit_scale, stc(i,3)/unit_scale, ...
                 ['  B', num2str(i)], 'FontSize', 10, 'FontWeight', 'bold');
        end
    end
    
    % 绘制移动信标
    if ~isempty(mov)
        plot3(mov(:,1)/unit_scale, mov(:,2)/unit_scale, mov(:,3)/unit_scale, ...
              'm--', 'LineWidth', 1.5, 'DisplayName', '移动信标');
        % 标记移动信标的起点
        plot3(mov(1,1)/unit_scale, mov(1,2)/unit_scale, mov(1,3)/unit_scale, ...
              'mo', 'MarkerSize', 8, 'HandleVisibility', 'off');
    end

    %% 3. 设置坐标轴标签与外观
    if strcmp(mode, 'xyz')
        u = p.Results.unit;
        xlabel(['东向 (E) [', u, ']']);
        ylabel(['北向 (N) [', u, ']']);
        zlabel(['天向 (U) [', u, ']']);
        title('直角坐标系 (Local Tangent Plane)');
    else
        xlabel('经度 (Longitude) [deg]');
        ylabel('纬度 (Latitude) [deg]');
        zlabel('高度 (Height) [m]');
        title('地理坐标系 (WGS84)');
        view(2); % 如果是经纬度，默认先看 2D 平面
    end
    
    legend('Location', 'best');
    axis equal; 
    set(gca, 'FontSize', 11, 'FontName', 'TimesSimSun');
end

function fig = myfigurestartup(width, height, type, varargin)
% 适配第三方复合字体 (如 TimesSimSun) 的强化版
% 功能：自动检测字体、锁定坐标轴属性、防止绘图覆盖

%% 1. 字体检测与容错
customFont = 'TimesSimSun'; 
allFonts = listfonts; % 获取系统已安装字体列表
if ~any(strcmpi(allFonts, customFont))
    warning('系统中未检测到字体 "%s"，将回退至宋体。', customFont);
    customFont = 'SimSun'; 
end

%% 2. 参数解析
switch type
    case 'paper'
        alw = 0.75; fsz = 8; lw = 1.2; msz = 6;
    case 'prese'
        alw = 1.2; fsz = 16; lw = 2.0; msz = 10;
    case 'zxy'
        alw = 0.75; fsz = 10; lw = 1.2; msz = 6;
end

%% 3. 创建 Figure
fig = figure('Color', 'w'); % 背景强制设为白色
screenSize = get(0, 'ScreenSize');
figWidth = width * 100; figHeight = height * 100;
set(fig, 'Position', [(screenSize(3)-figWidth)/2, (screenSize(4)-figHeight)/2, figWidth, figHeight]);

%% 4. 【核心强化】设置 Default 属性并锁定
% 使用 Default 属性可以确保后续 plot/xlabel 自动继承这些设置
set(fig, 'DefaultAxesFontName', customFont);
set(fig, 'DefaultTextFontName', customFont);
set(fig, 'DefaultLegendFontName', customFont);
set(fig, 'DefaultAxesFontSize', fsz);
set(fig, 'DefaultTextFontSize', fsz);
set(fig, 'DefaultLineLineWidth', lw);
set(fig, 'DefaultLineMarkerSize', msz);

%% 5. 坐标轴 (Axes) 深度配置
ax = gca;
hold(ax, 'on'); 
set(ax, 'LineWidth', alw, ...
        'Box', 'on', ...
        'TickDir', 'in', ...
        'XGrid', 'on', 'YGrid', 'on', ...
        'FontName', customFont, ... % 显式指定一次
        'TickLabelInterpreter', 'tex', ...
        'Layer', 'top'); % 保证刻度线不被图片遮挡

%% 6. 打印与导出优化
set(fig, 'Renderer', 'painters', ...
         'PaperUnits', 'inches', ...
         'PaperPosition', [0, 0, width, height], ...
         'PaperSize', [width, height]);
end
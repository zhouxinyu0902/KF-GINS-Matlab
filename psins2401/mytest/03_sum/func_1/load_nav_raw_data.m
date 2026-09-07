function [DistData_rw, PTSAGShip_rw, PTSAGHov_rw, PIXOG_rw, PTSAX_rw, cfg] = load_nav_raw_data(data_dir, isfig)
% LOAD_NAV_RAW_DATA 导入并初步解析深海组合导航原始数据
%
% 输入:
%   data_dir : 数据文件所在的文件夹路径 (例如 'data/'), 留空 '' 则默认当前目录
%   isfig    : 是否绘制初始的原始数据核对图 (1: 绘图, 0: 不绘图)
%
% 输出:
%   DistData_rw  : 整合后的舱内综合数据 (包含OCTANS, 罗盘, DVL, 深度, LBL结果等)
%   PTSAGShip_rw : USBL 母船定位数据
%   PTSAGHov_rw  : USBL 潜水器定位数据
%   PIXOG_rw     : USBL 传播时间与母船姿态数据
%   PTSAX_rw     : USBL 相对位移数据
%   cfg          : 包含时间配置和常数的结构体 (ATdelay, TimeUSBL 等)

if nargin < 1 || isempty(data_dir), data_dir = ''; end
if nargin < 2, isfig = 0; end

fprintf('开始导入原始导航数据...\n');

%% 1. 导入舱内数据 (AUV 传感器 & LBL)
file_main   = fullfile(data_dir, '1COMPS_2OCTANS_3DVL_4SHIP_5RANGE_6TSD.txt');
file_height = fullfile(data_dir, 'DVLheight_heighter.txt');
file_lbl    = fullfile(data_dir, 'POS20130628_LBL.txt');

fid1 = fopen(file_main, 'rt');
if fid1 == -1, error('❌ 找不到主数据文件: %s', file_main); end
fgets(fid1); DistData_rw = fscanf(fid1, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d/%d/%d %d:%d:%d\n', [23, inf]); fclose(fid1);

fid2 = fopen(file_height, 'rt');
if fid2 == -1, error('❌ 找不到高度计文件: %s', file_height); end
fgets(fid2); height = fscanf(fid2, '%f,%f,%d/%d/%d %d:%d:%d\n', [8, inf]); fclose(fid2);

fid3 = fopen(file_lbl, 'rt');
if fid3 == -1, error('❌ 找不到LBL文件: %s', file_lbl); end
DistData2 = fscanf(fid3, '%f,%f,%f,%d/%d/%d %d:%d:%d\n', [9, inf]); fclose(fid3);

% --- 矩阵重组 ---
DistData_rw(23:25, :) = DistData_rw(21:23, :); % 保护时间戳平移
DistData_rw(18:20, :) = DistData2(1:3, :);     % 填入长基线位置
DistData_rw(21:22, :) = height(1:2, :);        % 填入DVL高度
DistData_rw([1, 3], :) = DistData_rw([3, 1], :); % 交换第1行和第3行

%% 2. 导入 USBL 数据
file_ptsag = fullfile(data_dir, 'USBL_BOX-R-20130628-080921_PTXAG.log');
file_pixog = fullfile(data_dir, 'USBL_BOX-R-20130628-080921_PIXOG.log');
file_ptsax = fullfile(data_dir, 'USBL_BOX-R-20130628-080921_PTSAX.log');

fid = fopen(file_ptsag, 'rt');
if fid == -1, error('❌ 找不到 USBL PTSAG 文件: %s', file_ptsag); end
PTSAG_rw = fscanf(fid, '$PTSAG,#%d,%2d%2d%f,%d,%d,%d,%d,%2d%f,%c,%3d%f,%c,%X,%f,%d,%f*%X\n', [19, inf]); fclose(fid);

fid = fopen(file_pixog, 'rt');
if fid == -1, error('❌ 找不到 USBL PIXOG 文件'); end
PIXOG_rw = fscanf(fid, '$PIXOG,PPC,DETEC,%2d%2d%f,%d,%d,%d,%d,  %f,%f,%f,  %f,%f,%f,  %f,%f,%f,  %f,%f,%d,%f,   %f,%f,%d,%f,   %f,%f,%d,%f,   %f,%f,%d,     %f*%X\n', [33, inf]); fclose(fid);

fid = fopen(file_ptsax, 'rt');
if fid == -1, error('❌ 找不到 USBL PTSAX 文件'); end
PTSAX_rw = fscanf(fid, '$PTSAX,#%d,%2d%2d%f,%d,%d,%d,%d,%f,%f,%X,%f,%d,%f*%X\n', [15, inf]); fclose(fid);

% --- USBL 数据解析与清洗 ---
% PTSAG: 处理时间与经纬度半球
PTSAG_rw(end-2:end, :) = PTSAG_rw(2:4, :);
Lat = PTSAG_rw(9, :) + PTSAG_rw(10, :)/60;
Lat(PTSAG_rw(11, :) == 83) = -Lat(PTSAG_rw(11, :) == 83); % 'S' == 83
Lon = PTSAG_rw(12, :) + PTSAG_rw(13, :)/60;
Lon(PTSAG_rw(14, :) == 87) = -Lon(PTSAG_rw(14, :) == 87); % 'W' == 87
PTSAG_rw(9, :) = Lat;
PTSAG_rw(12, :) = Lon;
PTSAG_rw([2:7, 10:11, 13:14], :) = []; % 删除多余的度分秒字符行
PTSAG_rw(end, :) = floor(PTSAG_rw(end, :));

% 分离母船与潜水器
PTSAGShip_rw = PTSAG_rw(:, PTSAG_rw(2, :) == 0);
PTSAGHov_rw  = PTSAG_rw(:, PTSAG_rw(2, :) ~= 0);

% PIXOG & PTSAX: 处理时间置于末尾
PIXOG_rw(34:36, :) = PIXOG_rw(1:3, :);
PIXOG_rw(end, :) = round(PIXOG_rw(end, :));
PTSAX_rw(16:18, :) = PTSAX_rw(2:4, :);
PTSAX_rw(end, :) = round(PTSAX_rw(end, :));

%% 3. 时间配置参数打包
cfg.ATdelay = 38;
cfg.AdepC = 15.5;
cfg.TimeUSBL = [41458, 50258]; % 11:30:58--13:57:38
cfg.TimeLBL = cfg.TimeUSBL + cfg.ATdelay; % 11:31:36--13:58:16
cfg.TT_USBL = cfg.TimeUSBL(1) : 8 : cfg.TimeUSBL(end);
cfg.tt_usbl = 0 : 8 : (cfg.TT_USBL(end) - cfg.TT_USBL(1));

fprintf('数据导入完成！\n\n');

%% 4. (可选) 绘制初始传感器核对图
if isfig
    figure('Name', '传感器数据初始核对', 'Position', [100, 100, 800, 350]);
    subplot(1, 2, 1);
    plot(DistData_rw(3, :), 'b-', 'DisplayName', '罗盘'); hold on; grid on;
    plot(DistData_rw(6, :), 'r-', 'DisplayName', 'OCTANS');
    title('航向角走势对比'); ylabel('Heading (deg)'); legend('Location', 'best');

    subplot(1, 2, 2);
    plot(DistData_rw(6, :) - DistData_rw(3, :), 'k.', 'MarkerSize', 4);
    grid on; ylim([-10, 10]);
    title('航向角偏差 (OCTANS - Compass)'); ylabel('Diff (deg)');
end
end
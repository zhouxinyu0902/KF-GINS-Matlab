% 清理工作空间与初始化环境
clear; clc; 
glvs;
%% --- 阶段一：导入原始数据 ---
% (使用 '' 表示当前文件夹读取数据，1 表示显示初始检查图)
[DistData_rw, PTSAGShip_rw, PTSAGHov_rw, PIXOG_rw, PTSAX_rw, cfg] ...
    = load_nav_raw_data('data_1', 1);
%% --- 阶段二：时间同步与截取 ---
% 将阶段一的数据送入同步函数，1 表示显示时间对齐图
[DistData, TT_LBL, tt_lbl, USBL_sync] ...
    = sync_nav_time(DistData_rw, PTSAGShip_rw, PTSAGHov_rw, PTSAX_rw, PIXOG_rw, cfg, 1);
%% --- 阶段三：AUV 传感器精细预处理 ---
% 获取姿态、补偿后的深度和高精度的 DVL 速度！ (1 表示画图检查)
[compass, octans, depther, height, vxy, VXYZ_raw] = process_auv_sensors(DistData, tt_lbl, cfg, 1);
%% --- 阶段四：声学数据预处理与投影 ---
% 提取 USBL 换能器、潜水器坐标、LBL 参考真值等 (1 表示绘图检查)
[LBL_out, USBL_out] = process_acoustic_data(DistData, tt_lbl, USBL_sync, compass, depther, cfg, 1);
%% --- 阶段五：航位推算融合滤波  ---
[avp_LBL_DR,avp_usbl_raw,avp_lbl_raw]=integNAV(USBL_out,LBL_out,cfg,depther,octans,vxy,compass,tt_lbl);
%
% avp_lbl_raw  = LBL_out.avp_m;
% avp_usbl_raw = [zeros(length(cfg.tt_usbl), 6), d2r([USBL_out.LatHov', USBL_out.LonHov']),...
%     -depther(1:16:16*length(cfg.tt_usbl)), cfg.tt_usbl'];
% % 选择最高精度的 LBL+OCTANS+DVL 融合轨迹作为 Reference
% [avp_LBL_DR, ~] = AcousticDeadR('LBL', tt_lbl, avp_lbl_raw, d2r(0.2), avp_lbl_raw, octans, vxy, depther, 1);
%% --- 阶段六：深度反馈与传播时间补偿量化评估 ---
% 将 USBL 数据与最优基准 avp_LBL_DR 对比，验证补偿算法 (1表示出图)
Metrics = eval_acoustic_ranges(USBL_out, avp_LBL_DR, tt_lbl, cfg.tt_usbl, 1);
%% --- 阶段 7：数据重组与归档保存 ---
fprintf('开始组装兼容性变量并保存数据 (Phase 7)...\n');

% 1. 提取兼容老代码的变量名 (防止你后续的画图脚本报错)
HorizRangePropaT   = Metrics.PIXOG.Est_range(1:4, :);
HorizRangePropaTsm = Metrics.PIXOG.Est_range_sm;

% 组装矩阵 [纬度; 经度; 深度; 时间]
LatLonDepTran = [USBL_out.Transducer.Lat; USBL_out.Transducer.Lon; -USBL_out.Ship.Depth; cfg.tt_usbl];
LatLonDepShip = [USBL_out.Ship.Lat; USBL_out.Ship.Lon; -USBL_out.Ship.Depth; cfg.tt_usbl];
LatLonDepHov  = [USBL_out.LatHov; USBL_out.LonHov; -USBL_out.DepthHovPTSAX; cfg.tt_usbl];

% 还原 LBL 的变量名
BCN     = LBL_out.BCN;
RNG     = LBL_out.RNG;
RNG1     = LBL_out.RNG1;
avp_m   = LBL_out.avp_m;
LatLonShipCabin = LBL_out.LatLonShipCabin;
% 2. 准备保存目录
save_dir = 'D:\Github\PSINS\psins2401\mytest\03_sum\data_1';
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end
save_path = fullfile(save_dir, 'deep-sea_optimized.mat');

% 3. 执行保存 (兼顾了传统的散装变量和现代的结构体)
save(save_path, ...
    'avp_m', 'compass', 'octans', 'vxy', 'depther', 'BCN', 'RNG','RNG1', ...
    'HorizRangePropaT', 'HorizRangePropaTsm', ...
    'LatLonDepTran', 'LatLonDepHov', 'LatLonDepShip','LatLonShipCabin', ...
     'avp_LBL_DR', 'avp_lbl_raw', 'avp_usbl_raw', ...
    'LBL_out', 'USBL_out', 'Metrics', 'cfg'); 

fprintf('🎉 大功告成！所有数据已成功保存至: %s\n', save_path);
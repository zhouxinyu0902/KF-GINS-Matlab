%% 轨迹

%%
outputfolder = 'D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs-new\';
if ~exist(outputfolder, 'dir')
mkdir(outputfolder)
end
%% 可观测性分析
in_dir = 'F:/2_Data/惯导试验/实验数据/All_data/input5';
cfg = config_1(in_dir);
cfg.outputfolder ="D:\Github\KF-GINS-Matlab\new_惯导试验\exper\output5"; 
%% 对比轮换信标的导航优势（数据组5）
load(cfg.outputfolder+"\alt-B1-B2-B3\"+"observ.mat",'obslog','window_measurements')
% observability_results = analyze_beacon_observability(obslog, window_measurements, outputfolder);
results = analyze_beacon_observability_v2(obslog, window_measurements);
export_files = export_beacon_observability_results(results, outputfolder,...
    'ExportMat', false, ...
    'ExportComparisonFigure', false, ...
    'ExportBoxFigure', true,...
    'ExportSummaryCSV',true,...
    'ExportWindowCSV',false,...
    'FilePrefix','exper-5-obser');

% 径向误差对比
pathcell={cfg.outputfolder+"/PureIns.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3\"+"/ES-EKF-Fixed-B1.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3\"+"/ES-EKF-Fixed-B2.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3\"+"/ES-EKF-Fixed-B3.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3\"+"/ES-EKF-Alternating.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
title("Dataset 1")
exportgraphics(figall,outputfolder+"exper-5-observ-threewaysCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/exper-5-threewaysCMP.xlsx",'sheet', 1);

%% 高度对比分析
prefix = "exper-5-";
navFiles = {
    fullfile(cfg.outputfolder, "heightwayCMP/",'No-height update.nav')
    fullfile(cfg.outputfolder, "heightwayCMP/",'Direct assignment.nav')
    fullfile(cfg.outputfolder, "heightwayCMP/",'Measurement update.nav')
    };

[fig1,finalExcelData1] = calc_radial_error_gjb(cfg.truthpath,navFiles{:});
title("Dataset 1")
% 保存对比图片和表格
savePrefix = fullfile(outputfolder, prefix+"compare-none-assign-measUpdate");
% 保存图片
exportgraphics(fig1, savePrefix + ".png", "Resolution", 600);

% 保存表格
writecell(finalExcelData1, savePrefix + ".xlsx", "Sheet", "RMSE");
fprintf("对比图片已保存：%s\n", savePrefix + ".png");
fprintf("对比表格已保存：%s\n", savePrefix + ".xlsx");
results = compare_height_vertical_rmse( ...
    cfg.truthpath, navFiles, ...
    'MethodNames', ["No-height update","Direct assignment", "Measurement update"], ...
    'OutputFile', fullfile(outputfolder, 'height_vd_rmse.csv'), ...
    'FigureFile', fullfile(outputfolder, 'height_vd_error.png'));

%% 对比算法
pathcell={cfg.outputfolder+"/ESKF.nav",...
    cfg.outputfolder+"/Single-stage RTS.nav",...
    cfg.outputfolder+"/Proposed two-stage RTS.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
title("Dataset 1")
ylim([0,1000])
exportgraphics(figall,outputfolder+"exper-5-observ-algoCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/exper-5-algoCMP.xlsx",'sheet', 1);
%% 加上长基线
pathcell={cfg.outputfolder+"/ESKF-LBL.nav",...
    cfg.outputfolder+"/Single-stage RTS-LBL.nav",...
    cfg.outputfolder+"/Proposed two-stage RTS-LBL.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
title("Dataset 1")
ylim([0,200])
exportgraphics(figall,outputfolder+"exper-5-LBL-observ-algoCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/exper-5-LBL-algoCMP.xlsx",'sheet', 1);


%% 第二组数据
in_dir = 'F:/2_Data/惯导试验/实验数据/All_data/input6';
cfg = config_1(in_dir);
cfg.outputfolder ="D:\Github\KF-GINS-Matlab\new_惯导试验\exper\output6\";
%% %% 对比轮换信标的导航优势（数据组6）
load(cfg.outputfolder+"\alt-B1-B2-B3\"+"observ.mat",'obslog','window_measurements')
% observability_results = analyze_beacon_observability(obslog, window_measurements, outputfolder);
results = analyze_beacon_observability_v2(obslog, window_measurements);
export_files = export_beacon_observability_results(results, outputfolder,...
    'ExportMat', false, ...
    'ExportComparisonFigure', false, ...
    'ExportBoxFigure', true,...
    'ExportSummaryCSV',true,...
    'ExportWindowCSV',false,...
    'FilePrefix','exper-6-obser');
pathcell={cfg.outputfolder+"/PureIns.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3"+"/ES-EKF-Fixed-B1.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3"+"/ES-EKF-Fixed-B2.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3"+"/ES-EKF-Fixed-B3.nav",...
    cfg.outputfolder+"\alt-B1-B2-B3"+"/ES-EKF-Alternating.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
title("Dataset 2")
exportgraphics(figall,outputfolder+"exper-6-observ-threewaysCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/exper-6-threewaysCMP.xlsx",'sheet', 1);
%% 对比高度有效
prefix = "exper-6-";
navFiles = {
    fullfile(cfg.outputfolder,"heightwayCMP/", 'No-height update.nav')
    fullfile(cfg.outputfolder,"heightwayCMP/", 'Direct assignment.nav')
    fullfile(cfg.outputfolder,"heightwayCMP/", 'Measurement update.nav')
    };

[fig1,finalExcelData1] = calc_radial_error_gjb(cfg.truthpath,navFiles{:});
title("Dataset 2")
% 保存对比图片和表格
savePrefix = fullfile(outputfolder, prefix+ "compare-none-assign-measUpdate");
% 保存图片
exportgraphics(fig1, savePrefix + ".png", "Resolution", 600);

% 保存表格
writecell(finalExcelData1, savePrefix + ".xlsx", "Sheet", "RMSE");
fprintf("对比图片已保存：%s\n", savePrefix + ".png");
fprintf("对比表格已保存：%s\n", savePrefix + ".xlsx");
results = compare_height_vertical_rmse( ...
    cfg.truthpath, navFiles, ...
    'MethodNames', ["No-height update","Direct assignment", "Measurement update"], ...
    'OutputFile', fullfile(outputfolder, prefix+'height_vd_rmse.csv'), ...
    'FigureFile', fullfile(outputfolder, prefix+'height_vd_error.png'));
%% 对比算法
pathcell={cfg.outputfolder+"/ESKF.nav",...
    cfg.outputfolder+"/Single-stage RTS.nav",...
    cfg.outputfolder+"/Proposed two-stage RTS.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
ylim([0,500])
title("Dataset 2")
exportgraphics(figall,outputfolder+"exper-6-observ-algoCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/exper-6-algoCMP.xlsx",'sheet', 1);
%% 加上长基线
% 需要改一下颜色
pathcell={cfg.outputfolder+"/ESKF.nav",...
    cfg.outputfolder+"/ESKF-LBL.nav",...
    cfg.outputfolder+"/Single-stage RTS.nav",...
    cfg.outputfolder+"/Single-stage RTS-LBL.nav",...
    cfg.outputfolder+"/Proposed two-stage RTS.nav",...
    cfg.outputfolder+"/Proposed two-stage RTS-LBL.nav"
    };

[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
for i = 1:numel(finalExcelDataall)
    if isnumeric(finalExcelDataall{i}) && ~isempty(finalExcelDataall{i})
        finalExcelDataall{i} = round(finalExcelDataall{i}, 2);
    end
end
xlabel('Time(s)')
ylabel("Radial Error(m)")
title("Dataset 2")
ylim([0,200])
exportgraphics(figall,outputfolder+"exper-6-LBL-observ-algoCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/exper-6-LBL-algoCMP.xlsx",'sheet', 1);


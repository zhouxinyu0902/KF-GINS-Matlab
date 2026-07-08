%% 1.生成轨迹图

%%
outputfolder = 'D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs-new\';
if ~exist(outputfolder, 'dir')
    mkdir(outputfolder)
end
in_dir = "D:\Github\KF-GINS-Matlab\new_惯导试验\simu\input\";
cfg = config_simu(in_dir);
cfg.outputfolder ="D:\Github\KF-GINS-Matlab\new_惯导试验\simu\output\alt-B1-B2-B3\";
%% 2.水平几何分析(加上惯导)
load(cfg.outputfolder+"observ.mat",'obslog','window_measurements')
% observability_results = analyze_beacon_observability(obslog, window_measurements, outputfolder);
results = analyze_beacon_observability_v2(obslog, window_measurements);
export_files = export_beacon_observability_results(results, outputfolder,...
    'ExportMat', false, ...
    'ExportComparisonFigure', true, ...
    'ExportBoxFigure', true,...
    'ExportSummaryCSV',true,...
    'ExportWindowCSV',false,...
    'FilePrefix','simu-obser');
%% 径向误差对比
pathcell={cfg.outputfolder+"/PureIns.nav",...
    cfg.outputfolder+"/ES-EKF-Fixed-B1.nav",...
    cfg.outputfolder+"/ES-EKF-Fixed-B2.nav",...
    cfg.outputfolder+"/ES-EKF-Fixed-B3.nav",...
    cfg.outputfolder+"/ES-EKF-Alternating.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
exportgraphics(figall,outputfolder+"simu-observ-threewaysCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/simu-threewaysCMP.xlsx",'sheet', 1);
%% 算法对比
outputfolder11 = "D:\Github\KF-GINS-Matlab\new_惯导试验\simu\output\";
pathcell={outputfolder11+"/PureIns.nav",...
    outputfolder11+"/ES-EKF.nav",...
    outputfolder11+"/Single-stage RTS.nav",...
    outputfolder11+"/Proposed two-stage RTS.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
ylim([0,500])
title('Dataset 1')
exportgraphics(figall,outputfolder+"simu-observ-algoCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/simu-algoCMP.xlsx",'sheet', 1);
%%
in_dir = "D:\Github\KF-GINS-Matlab\new_惯导试验\simu\input1\";
cfg1 = config_simu(in_dir);
outputfolder11 = "D:\Github\KF-GINS-Matlab\new_惯导试验\simu\output1\";
pathcell={outputfolder11+"/PureIns.nav",...
    outputfolder11+"/ES-EKF.nav",...
    outputfolder11+"/Single-stage RTS.nav",...
    outputfolder11+"/Proposed two-stage RTS.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg1.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
ylim([0,500])
title('Dataset 2')
exportgraphics(figall,outputfolder+"simu1-observ-algoCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/simu1-algoCMP.xlsx",'sheet', 1);
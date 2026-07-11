in_dir = "D:\Github\KF-GINS-Matlab\new_惯导试验\fgo_navigation\input\";
cfg = config_simu(in_dir);
cfg.outputfolder ="D:\Github\KF-GINS-Matlab\new_惯导试验\fgo_navigation\output\";
pathcell={
    cfg.outputfolder+"/KF-RANGE-INS.nav",...
    cfg.outputfolder+"/FGO-RANGE-INS.nav",...
    cfg.outputfolder+"/FGO-RANGE-INS-HEIGHT-full15-accurate.nav",...
    };
[figa1,finalExcelData1] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});

% calc_error_gjb(pathcell{1},cfg.truthpath);
xlabel('Time(s)')
ylabel("Radial Error(m)")
%%
pathcell={ 
    cfg.outputfolder+"/KF-GNSS-INS.nav",...
    cfg.outputfolder+"/FGO-GNSS-INS-keyframes.nav",...
    cfg.outputfolder+"/Standard-FGO-GNSS-INS-keyframes.nav",...
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
%%

%% RTS平滑对比
pathcell={ 
    cfg.outputfolder+"/KF-RANGE-INS.nav",...
    cfg.outputfolder+"/KF-RANGE-INS-Proposed two-stage RTS.nav",...
    cfg.outputfolder+"/KF-RANGE-INS-Single-stage RTS.nav",...
    % cfg.outputfolder+"/FGO-RANGE-INS-smooth.nav"
    };
[figa2,finalExcelData2] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
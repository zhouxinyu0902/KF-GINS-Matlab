in_dir = "D:\Github\KF-GINS-Matlab\new_惯导试验\fgo_navigation\input\";
cfg = config_simu(in_dir);
cfg.outputfolder ="D:\Github\KF-GINS-Matlab\new_惯导试验\fgo_navigation\output\";
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
pathcell={ 
    cfg.outputfolder+"/FGO-RANGE-INS-keyframes.nav",...
    cfg.outputfolder+"/KF-RANGE-INS.nav",...
    };
[figa1,finalExcelData1] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
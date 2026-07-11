path="D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_line_N_four_quadrants\output_DR_RANGE";
pathcell={
    path+"/Origin-DR-1.nav",...
    path+"/DR-RANGE-1.nav",...
    path+"/DR-RANGE-2.nav",...
    path+"/DR-RANGE-3.nav",...
    path+"/DR-RANGE-4.nav"
    };
[figall,finalExcelDataall] = calc_radial_error_gjb(cfg.truthpath,pathcell{:});
xlabel('Time(s)')
ylabel("Radial Error(m)")
exportgraphics(figall,outputfolder+"simu-observ-threewaysCMP.png", 'Resolution', 600);
writecell(finalExcelDataall, outputfolder+"/simu-threewaysCMP.xlsx",'sheet', 1);
%%

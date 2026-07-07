path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_ArrayCenter_Trj\';
Calib = [path_used,'Calib-twice.nav'];
Origin = [path_used,'Origin.nav'];
True = [path_used,'True.nav'];
USBL = [path_used,'USBL.nav'];
truth = [path_used,'input_stage2\truth.nav'];
%%
[fig,finalExcelData] = calc_radial_error(truth,Origin,True,USBL,Calib);
legend('未补偿','真实位置','超短基线测定','本发明二次标定')
% 执行落地保存
woutputExcelName = [path_used,'导航系统径向误差统计报告.xlsx'];
writecell(finalExcelData, outputExcelName);
exportgraphics(fig, [path_used,'径向误差对比.png'], 'Resolution', 600);
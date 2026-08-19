cfg = ProcessConfig_exper();
output_dir = cfg.outputfolder;
pureins = fullfile(output_dir, 'PureIns.nav');
Origin = fullfile(output_dir, 'Origin.nav');
RTS1 = fullfile(output_dir, 'RTS-SingleSmooth.nav');
RTS2 = fullfile(output_dir, 'RTS-DoubleSmooth.nav');
Linear1 = fullfile(output_dir, 'Linear-SingleSmooth.nav');
Linear2 = fullfile(output_dir, 'Linear-DoubleSmooth.nav');

Originr = fullfile(output_dir, 'Origin-rad.nav');
RTS1r = fullfile(output_dir, 'RTS-SingleSmooth-rad.nav');
RTS2r = fullfile(output_dir, 'RTS-DoubleSmooth-rad.nav');
Linear1r = fullfile(output_dir, 'Linear-SingleSmooth-rad.nav');
Linear2r = fullfile(output_dir, 'Linear-DoubleSmooth-rad.nav');

Backwardrad = fullfile(output_dir, 'Backward-rad.nav');
BackwardRotaterad = fullfile(output_dir, 'BackwardRotate-rad.nav');

EKFrotate = fullfile(output_dir, 'EKFrotate.nav');

truth = cfg.truthpath;
%%
% [fig,finalExcelData] = calc_radial_error_gjb(truth,pureins,Origin,RTS1,RTS2,Linear1,Linear2);
[fig1,finalExcelData1] = calc_radial_error_gjb(truth,pureins,Origin,RTS1,RTS2,Linear1);
%% 没有纯惯导 m
[fig2,finalExcelData2] = calc_radial_error_gjb(truth,Origin,RTS1,RTS2);
% 没有纯惯导 rad
[fig3,finalExcelData3] = calc_radial_error_gjb(truth,Originr,RTS1r,RTS2r);
%%
[fig4,finalExcelData4] = calc_radial_error_gjb(truth,Originr,RTS2r,Backwardrad,BackwardRotaterad);
%%
[fig5,finalExcelData5] = calc_radial_error_gjb(truth,Originr,RTS2r,EKFrotate);

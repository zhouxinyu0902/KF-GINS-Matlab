%-------------优化惯导数据--------
% b_g =  cfg.gyrbiasstd ;
% b_a =  cfg.accbiasstd ;
% b_g =  navstate.gyrbias;
% b_a =  navstate.accbias;
% b_g = 0;
% b_a = 0;
param.sigma_g = cfg.gyrarw*expand;
param.sigma_a = cfg.accvrw*expand;
param.sigma_R = rangstd;

state.Cb0_n = navstate.cbn;
state.wnin = navstate.wnin;
beacon_pos = Rangedata(4:6)';
R_meas = Rangedata(3);

state.v0 = vel0;
state.p0 = pos0;
Hz = 1;
dtheta_tilde = sum(imudata(imuindex-99:imuindex,2:4),1)';
dv_tilde = sum(imudata(imuindex-99:imuindex,5:7),1)';
[dtheta_opt, dv_opt] = optimize_imu_incremental_1s(dtheta_tilde, dv_tilde, state, beacon_pos, R_meas, param, Hz);
dtheta = (dtheta_opt-dtheta_tilde)/100;
dv = (dv_opt-dv_tilde)/100;

% % state.p0 = navstate.pos;
% % state.v0 = navstate.vel;
% % Hz = 1/imudt;
% % dtheta_tilde = thisimu(2:4);
% % dv_tilde = thisimu(5:7);
% % [dtheta_opt, dv_opt] = optimize_imu_incremental(dtheta_tilde, dv_tilde,state, beacon_pos, R_meas, param, Hz);
% % dtheta = dtheta_opt-dtheta_tilde;
% % dv = dv_opt-dv_tilde;
% dtheta_1(rangeindex,:) = dtheta;
% dv_1(rangeindex,:) = dv;

thisimu(2:4, 1) = (thisimu(2:4, 1) + dtheta );
thisimu(5:7, 1) = (thisimu(5:7, 1) + dv );
%----------------------------
function kf = myDVLupdate(navstate,DVLdata,kf)
%MYDVLUPDATE 此处显示有关此函数的摘要
%   此处显示详细说明
velocity_d_rfu = DVLdata(2:4);
depth_meas_m =  DVLdata(5);
velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); -velocity_d_rfu(3)];
velocity_dvl_ned = navstate.cbn * velocity_body_frd;

depth_residual_m = navstate.pos(3) - (-depth_meas_m);
dvl_residual_mps = navstate.vel - velocity_dvl_ned;

Z = [depth_residual_m; dvl_residual_mps];
H = zeros(4, kf.RANK);
H(1, 3) = 1;
H(2:4, 4:6) = eye(3);
R = diag([0.2^2; repmat(0.01^2, 3, 1)]);
kf.Z = Z;
kf.Zkk_1 = H * kf.x;
K = kf.P * H' / (H * kf.P * H' + R);
%% 更新协方差和状态量

kf.x = kf.x + K * (Z - kf.Zkk_1 );
kf.P =(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
end


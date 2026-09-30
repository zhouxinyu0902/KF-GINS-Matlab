function kf = myLBLDVLupdate(navstate, DVL , LBL, kf)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
    % function kf = myRangeUpdate(navstate, Rangedata, depthdata, kf)
% Rangedata:4：6是信标的位置，3是水平距离，2是斜距，1是时间
% depthdata:4：2是深度，1是时间
% % 根据惯导和信标位置计算水平距离

%% 使用非线性一步预测量测值
% 直接计算
glvs
R1 = [2,2]/glv.Re;
%%
velocity_d_rfu = DVL(2:4);
velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); -velocity_d_rfu(3)];
velocity_dvl_ned = navstate.cbn * velocity_body_frd;
Z = [
    navstate.pos(1:2) - LBL(2:3)'
    navstate.pos(3) - (-DVL(5));
    navstate.vel - velocity_dvl_ned;
    ];
kf.Z = Z;
% 量测矩阵和噪声矩阵
R = diag([R1,0.2,0.01,0.01,0.01].^2);
H = zeros(6, kf.RANK);
H(1:3, 1:3) = eye(3);
H(4:6 , 4:6) = eye(3);
kf.Zkk_1 = H * kf.x;
K = kf.P * H' / (H * kf.P * H' + R);
%% 更新协方差和状态量
kf.x = kf.x + K * (Z - kf.Zkk_1 );
kf.P =(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
end


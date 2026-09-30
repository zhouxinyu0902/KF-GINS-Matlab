function kf = myRangeDVLupdate(navstate, DVL , Rangedata, kf)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
    % function kf = myRangeUpdate(navstate, Rangedata, depthdata, kf)
% Rangedata:4：6是信标的位置，3是水平距离，2是斜距，1是时间
% depthdata:4：2是深度，1是时间
param = Param();
% % 根据惯导和信标位置计算水平距离
bcn = Rangedata(4:6)';
%% 使用非线性一步预测量测值
% 直接计算
[rm, rn] = getRmRn(bcn(1) , param);
h = bcn(3);
DR = diag([rm + h, (rn + h)*cos(bcn(1)), -1]);
delta_pos = ( DR * (navstate.pos - bcn))';
HorizR = sqrt(sum(delta_pos(:,1:2).^2,2));
%%
velocity_d_rfu = DVL(2:4);
velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); -velocity_d_rfu(3)];
velocity_dvl_ned = navstate.cbn * velocity_body_frd;
Z = [
    navstate.pos(3) - (-DVL(5));
    HorizR-Rangedata(3);
    navstate.vel - velocity_dvl_ned;
    ];
kf.Z = Z;
% 量测矩阵和噪声矩阵
R = diag([0.2,5,0.01,0.01,0.01].^2);
H = zeros(5, kf.RANK);
b = (navstate.pos'-bcn')*(diag([rm + h, (rn + h)*cos(bcn(1)), -1])^2)/HorizR;
H(1, 3) = 1;
H(2, 1:2) = b(1:2);
H(3:5 , 4:6) = eye(3);
kf.Zkk_1 = H * kf.x;
K = kf.P * H' / (H * kf.P * H' + R);
%% 更新协方差和状态量
kf.x = kf.x + K * (Z - kf.Zkk_1 );
kf.P =(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
end


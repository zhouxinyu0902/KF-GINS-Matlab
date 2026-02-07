%% 距离更新
function kf = myRange_azi_pos_Update(navstate, Rangedata, depthdata, kf)
% Rangedata:4：6是信标的位置，3是水平距离，2是斜距，1是时间，7是方位角
% depthdata:4：2是深度，1是时间
global rangstd
global depstd
global posstd
param = Param();
bcn = Rangedata(4:6)';
[rm, rn] = getRmRn(bcn(1) , param);
h_earth = bcn(3);
dN = (bcn(1) - navstate.pos(1)) * (rm + h_earth);
dE = (bcn(2) - navstate.pos(2)) * (rn + h_earth) * cos(navstate.pos(1));


HorizR2 = dN^2 + dE^2;
HorizR_cal = sqrt(HorizR2);

azi_cal = pos2azimuth(-[dE,dN],[0,0],r2d(navstate.att(3))); % 计算方位角，单位为°

pos_xy = calc_position_from_beacon([0,0],Rangedata(3),Rangedata(7),r2d(navstate.att(3)));
kf.pos_rad1 = dxyz2pos([pos_xy,0],Rangedata(4:6)');

res_range = HorizR_cal - Rangedata(3); % 水平距离残差
res_depth = navstate.pos(3) - depthdata(2); % 深度残差
res_azi   = azi_cal - Rangedata(7); % 方位角残差 (deg)
res_azi = mod(res_azi + 180, 360) - 180;% [修正] 角度归一化 (-180 ~ 180)
res_pos = navstate.pos(1:2) - kf.pos_rad1(1:2)';

Z = [res_range;
    res_depth;
    res_azi;
    res_pos];
% Z = [res_range;
%     res_depth;];
kf.Z = Z;

%% 量测矩阵和噪声矩阵
% azistd = 0.4-abs(abs(azi_cal)-90)/90*0.3;
azistd = 0.15;
vk = [rangstd,depstd,azistd,posstd/(rm + h_earth),posstd/((rn + h_earth) * cos(navstate.pos(1)))];
% vk = [rangstd,depstd,azimustd];

% vk = [rangstd,depstd];
R = diag(vk.^2);
H = zeros(5, kf.RANK);
% b = (navstate.pos'-bcn')*(diag([rm + h, (rn + h)*cos(bcn(1)), -1])^2)/HorizR_cal;
% H(1, 1:2) = b(1:2);
% H(2, 3) = 1;
% H(3, 1:3) = c; H(3, 9) = -1;
% 
% kf.Zkk_1 = H * kf.x;


H(1, 1) = (-dN / HorizR_cal) * (rm + h_earth);     
H(1, 2) = (-dE / HorizR_cal) * (rn + h_earth) * cos(navstate.pos(1));
H(2, 3) = 1;
deg_per_rad = 180 / pi;
H(3, 1) = (dE / HorizR2) * (rm + h_earth) * deg_per_rad;
H(3, 2) = (-dN / HorizR2) * (rn + h_earth) * cos(navstate.pos(1)) * deg_per_rad;
H(3, 9) = -1 * deg_per_rad;
H(4, 1) = 1;
H(5, 2) = 1;
%% 更新协方差和状态量

K = kf.P * H' / (H *kf.P * H' + R);
kf.x = kf.x + K * Z ;
kf.P =(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';

end

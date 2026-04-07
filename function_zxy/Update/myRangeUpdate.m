%% 距离更新
function kf = myRangeUpdate(navstate, Rangedata, depthdata, kf)
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
delta_pos = ( DR * (navstate.pos-bcn))';
HorizR = sqrt(sum(delta_pos(:,1:2).^2,2));
%%
Z = [HorizR-Rangedata(3);
    navstate.pos(3)-depthdata(2)];
kf.Z = Z;
% 量测矩阵和噪声矩阵
vk = [kf.rangstd,kf.depthstd];
R = diag(vk.^2);
H = zeros(2, kf.RANK);
b = (navstate.pos'-bcn')*(diag([rm + h, (rn + h)*cos(bcn(1)), -1])^2)/HorizR;
H(1, 1:2) = b(1:2);
H(2, 3) = 1;
kf.Zkk_1 = H * kf.x;
%%
% Z = HorizR-Rangedata(3);
% kf.Z = Z;
% vk = kf.rangstd;
% R = diag(vk.^2);
% H = zeros(1, kf.RANK);
% b = (navstate.pos'-bcn')*(diag([rm + h, (rn + h)*cos(bcn(1)), -1])^2)/HorizR;
% H(1, 1:2) = b(1:2);
% kf.Zkk_1 = H * kf.x;
%%
K = kf.P * H' / (H * kf.P * H' + R);
% K(7:end,:) = zeros(size(K(7:end,:)));
%% 更新协方差和状态量


kf.x = kf.x + K * (Z - kf.Zkk_1 );
kf.P =(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';

% UKF更新x和P
% alpha = 1e-3; beta = 2; kappa = 0;
% n = length(kf.x);
% lambda = alpha^2*(n+kappa) - n;
% gamma = sqrt(n+lambda);
% Wm = [lambda/gamma^2; repmat(1/(2*gamma^2),2*n,1)];
% Wc = [Wm(1)+(1-alpha^2+beta); Wm(2:end)];
% sPxx = gamma*chol(kf.P)';    % Choleskey decomposition
% xn = repmat(kf.x,1,n);
% X = [kf.x, xn+sPxx, xn-sPxx];
% Y(:,1) = H*X(:,1); m=length(Y); zkk_1 = Wm(1)*Y(:,1);
% Y = repmat(Y,1,2*n+1);
% for k=2:1:2*n+1     % Sigma points nolinear propagation
%     Y(:,k) = H*X(:,k);
%     zkk_1 = zkk_1 + Wm(k)*Y(:,k);
% end
% Pyy = zeros(m); Pxy = zeros(n,m);
% for k=1:1:2*n+1
%     yerr = Y(:,k)-zkk_1;
%     Pyy = Pyy + Wc(k)*(yerr*yerr');  % variance
%     xerr = X(:,k)-kf.x;
%     Pxy = Pxy + Wc(k)*xerr*yerr';  % covariance
% end
% K = Pxy / (Pyy + R);
% kf.x = kf.x + K*(Z - zkk_1 );
% kf.P = kf.P - K*(Pyy + R)*K';

%% 反馈后，再次计算残差
pos_new = navstate.pos-kf.x(1:3);
delta_pos =( DR * (pos_new-bcn))';
SlantR = sqrt(sum(delta_pos(:,1:3).^2,2));
HorizR_new = sqrt(SlantR.^2-delta_pos(:,3).^2);
kf.Znew = HorizR_new-Rangedata(3);
end

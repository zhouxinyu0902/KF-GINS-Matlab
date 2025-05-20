%% 距离更新
function kf = my2RangeUpdate(navstate, Rangedata1, Rangedata2, depthdata, kf)
param = Param();
% measurement innovation
% 信标位置
bcn1=Rangedata1(4:6)';
bcn1(1:2)=bcn1(1:2)*param.D2R;
[~,range_ins1]=caldot2dot(navstate.pos',bcn1'); % 根据惯导和信标位置计算水平距离

bcn2=Rangedata2(4:6)';
bcn2(1:2)=bcn2(1:2)*param.D2R;
[~,range_ins2]=caldot2dot(navstate.pos',bcn2'); 

Z = [Rangedata1(3)-range_ins1;
    Rangedata2(3)-range_ins2;
    depthdata(2)-navstate.pos(3)];

% measurement matrix and noise matrix
vk = [1,1,0.2];
R = diag(vk.^2);
H = zeros(3, kf.RANK);
b1 = (bcn1'-navstate.pos')*diag([(navstate.Rm + navstate.pos(3))^2,...
    ((navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)))^2,...
    1])/range_ins1;
H(1, 1:2) = b1(1:2);

b2 = (bcn2'-navstate.pos')*diag([(navstate.Rm + navstate.pos(3))^2,...
    ((navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)))^2,...
    1])/range_ins2;
H(2, 1:2) = b2(1:2);
H(3, 3) = -1;


% update covariance and state vector

% % 非线性更新一步量测预测值---无用
% lonlath=navstate.pos(1:3)-kf.x(1:3);
% h=navstate.pos(3)-kf.x(3);
% [~,range]=caldot2dot(lonlath',bcn');
% kf.Zkk_1 = [Rangedata(3)-range;
%             depthdata(2)-h];

kf.Zkk_1 = H * kf.x;
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K*(Z - kf.Zkk_1 );
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

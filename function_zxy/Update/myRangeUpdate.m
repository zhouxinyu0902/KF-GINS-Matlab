%% 距离更新
function kf = myRangeUpdate(navstate, Rangedata, depthdata, kf)
% Rangedata:4：6是信标的位置，3是水平距离，2是斜距，1是时间
% depthdata:4：2是深度，1是时间
global rangstd
global depstd
param = Param();

% % 根据惯导和信标位置计算水平距离
bcn = Rangedata(4:6)';
%% 使用非线性一步预测量测值
% 直接计算
[rm, rn] = getRmRn(bcn(1) , param);
h = bcn(3);
DR = diag([rm + h, (rn + h)*cos(bcn(1)), -1]);
delta_pos =( DR * (navstate.pos-bcn))';
SlantR=sqrt(sum(delta_pos(:,1:3).^2,2));
HorizR=sqrt(sum(delta_pos(:,1:2).^2,2));
Z = [HorizR-Rangedata(3);
    navstate.pos(3)-depthdata(2)];
kf.Z=Z;
% disp(Z)
%% 非线性更新一步量测预测值---无用
% lonlath=navstate.pos(1:3)-kf.x(1:3);
% h=navstate.pos(3)-kf.x(3);
% [~,range]=caldot2dot(lonlath',bcn');
% kf.Zkk_1 = [Rangedata(3)-range;
%             depthdata(2)-h];
% Z = [Rangedata(3);
%     depthdata(2)];
% [~,range_ins_non]=caldot2dot((navstate.pos-kf.x(1:3))',bcn');
% kf.Zkk_1 =[range_ins_non;navstate.pos(3)-kf.x(3)];

%% 调整增益
% meas=15*(abs(Z(1))/700)+15;
% disp([Z(1),meas]);
% format bank
% meas=rangstd;
% meas=5+(1-(navstate.time-122235)/5000)*10;
% disp(meas)
% meas=15;
% disp('-----')

%% 量测矩阵和噪声矩阵
vk = [rangstd,depstd];
R = diag(vk.^2);
H = zeros(2, kf.RANK);
b = (navstate.pos'-bcn')*(diag([rm + h, (rn + h)*cos(bcn(1)), -1])^2)/HorizR;
H(1, 1:2) = b(1:2);
H(2, 3) = 1;

kf.Zkk_1 = H * kf.x;
%% 自适应因子
[alpha, d_squared,chi2_threshold, is_anomaly] = calculate_adaptive_factor(Z(1), H(1,:), kf.P, R(1), 0.05);
kf.alpha = alpha;
kf.d_squared = d_squared;
kf.chi2_threshold = chi2_threshold;
kf.is_anomaly = is_anomaly;


% kf.P = alpha * kf.P;
%% 更新协方差和状态量
alpha=1;
K = alpha * kf.P * H' / (H * alpha * kf.P * H' + R);
% K = kf.P * H' / (H *kf.P * H' + R);
kf.x = kf.x + K*(Z - kf.Zkk_1 );
kf.P =(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
% disp('update')
% for i=1:15
%     disp(diag(kf.P(i,i)))
% end

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
SlantR=sqrt(sum(delta_pos(:,1:3).^2,2));
HorizR_new=sqrt(SlantR.^2-delta_pos(:,3).^2);
kf.Znew=HorizR_new-Rangedata(3);
end
function [alpha, d_squared,chi2_threshold, is_anomaly] = calculate_adaptive_factor(innovation, H, P_pred, R, significance_level)
% 基于新息协方差的统计检验计算自适应因子α
%
% 输入:
%   innovation    - 新息向量 (观测残差), z - H*x_pred
%   H             - 观测矩阵
%   P_pred        - 预测状态协方差矩阵
%   R             - 观测噪声协方差矩阵
%   significance_level - 显著性水平 (默认 0.05, 即95%置信度)
%
% 输出:
%   alpha       - 计算得到的自适应因子 (α ≥ 1)
%   d_squared   - 标准化新息的马氏距离平方
%   is_anomaly  - 标记当前新息是否统计异常

% 参数检查与默认值
if nargin < 5
    significance_level = 0.05; % 默认95%置信度
end

% 1. 计算理论新息协方差矩阵 S
S = H * P_pred * H' + R;

% 2. 计算标准化新息的马氏距离平方 (d²)
%    使用更稳定的线性求解方法替代直接求逆
d_squared = innovation' * (S \ innovation);

% 3. 获取卡方分布阈值
n = length(innovation); % 观测维度 = 卡方分布自由度
chi2_threshold = chi2inv(1 - significance_level, n);

% 4. 统计检验与自适应因子计算
if d_squared <= chi2_threshold
    % 情况1: 新息在统计预期范围内 → 模型可靠
    alpha = 1.0;
    is_anomaly = false;
else
    % 情况2: 新息显著异常 → 模型不可靠，需要调整
    % 核心公式: α = d² / χ²_threshold
    alpha = d_squared / chi2_threshold;
    is_anomaly = true;

    % 可选: 对α进行平滑或限幅，防止过度调整
    alpha = max(1.0, min(alpha, 10)); % 通常限制在1到5之间
end

% 输出诊断信息
% fprintf('自适应因子计算: 新息残差=%.3f,d²=%.3f, χ²阈值=%.3f, α=%.3f, 异常=%d\n', ...
%     innovation, d_squared, chi2_threshold, alpha, is_anomaly);
end
%% 距离辅助航位推算
function kf= myekf(type,varargin)
%MYEKF 将四个模块合成一个
switch(type)
    case 'init'
        [ts,x0,dx0,vk,rk]=setvals(varargin);
        kf=[];
        kf.m=length(x0);
        kf.n=length(rk);
        kf.xk=zeros(kf.m,1);
        kf.xk=x0;
        kf.xkk_1=zeros(kf.m,1);
        kf.Qt = diag(vk).^2;
        kf.Rk = diag(rk).^2;
        kf.Hk = zeros(kf.n,kf.m);
        kf.Pxk = diag(dx0).^2 ;
        kf.Phikk_1=zeros(kf.m,kf.m);
        kf.yk=zeros(kf.n,1);
        kf.Fk=zeros(kf.m,kf.m);
        kf.Ft=zeros(kf.m,kf.m);
        kf.ts=ts;
        kf.coef_fb=1;
        kf.MK=zeros(kf.m,kf.m);
    case 'fk'
        [kf,dr]=setvals(varargin);
        % 获取系统转移矩阵
        Ft=zeros(kf.m,kf.m);
        VE=dr.vn(1);
        VN=dr.vn(2);
        phi = dr.att(3);
        % 重新推导
        if kf.m == 4
            Ft(3,1) = VN/dr.eth.RMh;
            Ft(3,2) = VE/dr.eth.RMh;
            Ft(4,1) = VE/dr.eth.clRNh;
            Ft(4,2) = -VN/dr.eth.clRNh;
            Ft(4,3) = VE*dr.eth.sl/(dr.eth.clRNh*dr.eth.cl);
        elseif kf.m==5
            pp = 2 * phi;
            Ft(4,1) = VN/dr.eth.RMh;
            Ft(4,2) = VE*cos(pp)/dr.eth.RMh;
            Ft(4,3) = VE*sin(pp)/dr.eth.RMh;

            Ft(5,1) = VE/dr.eth.clRNh;
            Ft(5,2) = -VN*cos(pp)/dr.eth.clRNh;
            Ft(5,3) = -VN*sin(pp)/dr.eth.clRNh;
            Ft(5,4) = VE*dr.eth.sl/(dr.eth.clRNh*dr.eth.cl);
        elseif kf.m==6
            Ft(5,1) = VN/dr.eth.RMh;
            Ft(5,2) = VE/dr.eth.RMh;
            Ft(5,3) = sin(phi)/dr.eth.RMh;
            Ft(5,4) = cos(phi)/dr.eth.RMh;

            Ft(6,1) = VE/dr.eth.clRNh;
            Ft(6,2) = -VN/dr.eth.clRNh;
            Ft(6,3) = cos(phi)/dr.eth.clRNh;
            Ft(6,4) = -sin(phi)/dr.eth.clRNh;
            Ft(6,5) = VE*dr.eth.sl/(dr.eth.clRNh*dr.eth.cl);
        elseif kf.m==7
            Ft(6,1) = VN/dr.eth.RMh;
            Ft(6,2) = VE*cos(2*phi)/dr.eth.RMh;
            Ft(6,3) = VE*sin(2*phi)/dr.eth.RMh;
            Ft(6,4) = sin(phi)/dr.eth.RMh;
            Ft(6,5) = cos(phi)/dr.eth.RMh;

            Ft(7,1) = VE/dr.eth.clRNh;
            Ft(7,2) = -VN*cos(2*phi)/dr.eth.clRNh;
            Ft(7,3) = -VN*sin(2*phi)/dr.eth.clRNh;
            Ft(7,4) = cos(phi)/dr.eth.clRNh;
            Ft(7,5) = -sin(phi)/dr.eth.clRNh;
            Ft(7,6) = VE*dr.eth.sl/(dr.eth.clRNh*dr.eth.cl);
        end
        % 离散
        kf.Phikk_1= eye(kf.m) + Ft*dr.ts;% + Fk*Fk*0.5;
    case 'hk'
        [kf,dr,hktype]=setvals(varargin);
        switch(hktype)
            case 'range'
                if isfield(kf,'r_dr')
                    b=(dr.pos'-dr.beacon)*diag([dr.eth.RMh^2,dr.eth.clRNh^2,1])/kf.r_dr;
                end
                if isfield(kf,'Rrng')
                    b=(dr.pos'-dr.beacon)*diag([dr.eth.RMh^2,dr.eth.clRNh^2,1])/kf.Rrng;
                end
                if kf.m==4
                    kf.Hk=[zeros(1,2),b(1:2)];
                elseif kf.m==5
                    kf.Hk=[zeros(1,3),b(1:2)];
                elseif kf.m==6
                    kf.Hk=[zeros(1,4),b(1:2)];
                elseif kf.m==7
                    kf.Hk=[zeros(1,5),b(1:2)];
                end
            case 'LBL'
                kf.Hk=[0,0,1,0;0,0,0,1];
            case '2range'
                if isfield(kf,'r_dr') % 水平距离
                    b1 = (dr.pos'-dr.beacon1)*diag([dr.eth.RMh^2,dr.eth.clRNh^2,1])/kf.r_dr(1);
                    b2 = (dr.pos'-dr.beacon2)*diag([dr.eth.RMh^2,dr.eth.clRNh^2,1])/kf.r_dr(2);
                end
                if isfield(kf,'Rrng') % 斜距
                    b1 = (dr.pos'-dr.beacon)*diag([dr.eth.RMh^2,dr.eth.clRNh^2,1])/kf.Rrng;
                end
                kf.Hk=[zeros(1,2),b1(1:2);zeros(1,2),b2(1:2)];
        end
        kf.ykk_1=kf.Hk*kf.xkk_1;
    case 'algo'
        if length(varargin)==3
            [kf,updatetype,Adap]=setvals(varargin);
        else
            [kf,updatetype]=setvals(varargin);
            Adap='EKF';
        end
        % 扩展卡尔曼滤波算法
        switch updatetype
            case 'T'
                % 一步预测
                kf.xkk_1 = kf.Phikk_1*kf.xk;
                % kf.Qk=kf.Qt*kf.ts/2; % 这种最准
                kf.Qk=(kf.Qt+kf.Phikk_1*kf.Qt*kf.Phikk_1')*kf.ts/2; % Qt离散化
                % kf.Qk=kf.Phikk_1*kf.Qt*kf.Phikk_1'*kf.ts; % Qt离散化
                kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Qk;
                kf.xk=kf.xkk_1;
                kf.Pxk=kf.Pxkk_1;
            case 'M'
                kf.Mk=kf.Phikk_1'* kf.Hk' *(kf.Rk)^-1 * kf.Hk * kf.Phikk_1;
                kf.MK=kf.MK+kf.Mk;

                kf.Lk=kf.Hk'*(kf.Rk)^-1*kf.Hk;

                % 滤波增益K
                % kf.Pxykk_1 = kf.alpha* kf.Pxkk_1*kf.Hk';    kf.Pykk_1 =kf.alpha* kf.Hk*kf.Pxykk_1 + kf.Rk;
                % kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;

                if strcmpi(Adap, 'AEKF')
                    % 新息向量
                    innovation = kf.yk - kf.ykk_1;
                    % 预测协方差矩阵
                    P_pred = kf.Pxkk_1;
                    Hk = kf.Hk;
                    R_nom = kf.Rk;
                    significance_level = 0.2;  % 95% 置信度，可根据实际调整

                    % 计算理论新息协方差 S
                    S = Hk * P_pred * Hk' + R_nom;
                    % 添加微小正则化保证数值稳定
                    [m, ~] = size(S);
                    S_reg = S + 1e-8 * eye(m);

                    % 马氏距离平方
                    d_squared = innovation' * (S_reg \ innovation);
                    % 卡方自由度 = 观测维数
                    dof = length(innovation);
                    chi2_threshold = chi2inv(1 - significance_level, dof);

                    if d_squared <= chi2_threshold
                        kf.alpha = 1.0;
                    else
                        % 自适应因子公式：α = d² / χ²_threshold
                        kf.alpha = d_squared / chi2_threshold;
                        % 限制最大缩放倍数，防止过度放大
                        kf.alpha = min(kf.alpha, 1e6);
                    end
                else
                    kf.alpha = 1.0;
                end
                % % 滤波增益K
                kf.Pxykk_1 = kf.Pxkk_1 * kf.Hk';
                kf.Pykk_1 = kf.Hk * kf.Pxykk_1 + 1/kf.alpha *kf.Rk;
                kf.Kk = kf.Pxykk_1 * kf.Pykk_1^-1;


                % 更新P和X
                if   strcmpi(Adap, 'UKF')
                    % UKF更新步骤
                    [kf.xk, kf.Pxk] = ukf_update(kf.xkk_1, kf.Pxkk_1, ...
                        kf.yk, kf.Hk, kf.Rk, ...
                        kf.Phikk_1);
                else
                    kf.Pxk = kf.Pxkk_1 - kf.Kk * kf.Pykk_1*kf.Kk';
                    kf.xk = kf.xkk_1 + kf.Kk * (kf.yk-kf.ykk_1);
                end
            case 'B'
                % 一步预测
                kf.xkk_1 = kf.Phikk_1*kf.xk;
                % kf.Qk=kf.Qt*kf.ts/2; % 这种最准
                kf.Qk=(kf.Qt+kf.Phikk_1*kf.Qt*kf.Phikk_1')*kf.ts/2; % Qt离散化
                % kf.Qk=kf.Phikk_1*kf.Qt*kf.Phikk_1'*kf.ts; % Qt离散化
                kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Qk;

                % 滤波增益K
                kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
                kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
                % 更新P和X
                kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';
                kf.xk = kf.xkk_1 + kf.Kk*(kf.yk-kf.ykk_1);
        end
end
end

% 输出诊断信息
% fprintf('自适应因子计算: 新息残差=%.3f,d²=%.3f, χ²阈值=%.3f, α=%.3f, 异常=%d\n', ...
%     innovation, d_squared, chi2_threshold, alpha, is_anomaly);
function [xk, Pxk] = ukf_update(xkk_1, Pxkk_1, yk, Hk, Rk, Phikk_1)
% 输入:
% xkk_1: 预测状态, Pxkk_1: 预测协方差, yk: 观测值
% Hk: 观测矩阵/函数句柄, Rk: 观测噪声协方差, Phikk_1: 状态转移相关

n = length(Phikk_1);      % 状态维数
m = length(yk);         % 观测维数

%% 1. UKF 参数配置 (标准设置)
alpha = 1e-3;           % 决定 Sigma 点的展布
ki = 0;
beta = 2;               % 高斯分布下 2 为最优
lambda = alpha^2 * (n + ki) - n;

% 计算权重系数
W_m = zeros(2*n+1, 1);
W_c = zeros(2*n+1, 1);
W_m(1) = lambda / (n + lambda);
W_c(1) = W_m(1) + (1 - alpha^2 + beta);
for i = 2:2*n+1
    W_m(i) = 1 / (2 * (n + lambda));
    W_c(i) = W_m(i);
end

%% 2. 生成 Sigma 点 (基于预测分布)
% 注意：如果 Pxkk_1 失去正定性，可在此做数值修正
sqrtP = chol((n + lambda) * Pxkk_1, 'lower');
X_sigmas = [xkk_1, xkk_1 + sqrtP, xkk_1 - sqrtP];

%% 3. 观测预测 (通过 Hk 映射 Sigma 点)
Y_sigmas = zeros(m, 2*n+1);
y_hat = zeros(m, 1);

for i = 1:2*n+1
    % 如果 Hk 是矩阵则直接相乘，如果是函数句柄则调用
    if isa(Hk, 'function_handle')
        Y_sigmas(:, i) = Hk(X_sigmas(:, i));
    else
        Y_sigmas(:, i) = Hk * X_sigmas(:, i);
    end
    y_hat = y_hat + W_m(i) * Y_sigmas(:, i);
end

%% 4. 计算协方差与增益
Pyk = Rk;               % 观测预测协方差
Pxy = zeros(n, m);      % 状态-观测互协方差

for i = 1:2*n+1
    y_diff = Y_sigmas(:, i) - y_hat;
    x_diff = X_sigmas(:, i) - xkk_1;

    Pyk = Pyk + W_c(i) * (y_diff * y_diff');
    Pxy = Pxy + W_c(i) * (x_diff * y_diff');
end

%% 5. 最终更新
K = Pxy / Pyk;                  % 卡尔曼增益
xk = xkk_1 + K * (yk - y_hat);  % 更新状态
Pxk = Pxkk_1 - K * Pyk * K';    % 更新协方差
end

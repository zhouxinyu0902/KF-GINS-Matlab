%% dt为周期的MEMS反向惯性推算
thisimu_60 = lastimu;
lastimu_60 = thisimu;
laststate_60 = navstate;
navstate_60 = navstate;
% 存住初始值
nav = zeros(11, 1);
nav(2, 1) = laststate_60.time;
nav(3:5, 1) = [laststate_60.pos(1) * param.R2D; laststate_60.pos(2) * param.R2D; laststate_60.pos(3)];
nav(6:8, 1) = laststate_60.vel;
nav(9:11, 1) = laststate_60.att * param.R2D;
navdt(dt*100,:) = nav;
% 继承KF
kf60 = kf;
% 为RTS平滑存值，后向的一些初始参数记录
xb(dt*100, :)  = kf60.x;
Xbnav_seq(dt*100, :) = [laststate_60.time; laststate_60.pos; laststate_60.vel; laststate_60.att];
Pb(:, :, dt*100) = kf60.P;

rangeindex_60 = floor(imuindex/dt/100)*dt-1;
% 调整参数
% kf60.P = kf.P*10000;
for rr=1:dt*100-1
    if  rangeindex_60 > 0 && abs(rangedata(rangeindex_60,1)-navstate_60.time)<0.008 
        kf60 = myRangeUpdate(navstate_60, rangedata(rangeindex_60,:), height(rangeindex_60*100,:), kf60);
        [kf60, laststate_60] = myErrorFeedback_range(kf60, navstate_60);

        navstate_60 = InsMechBackward(laststate_60, lastimu_60, thisimu_60);
        kf60 = myInsPropagate_15state(navstate_60, thisimu_60, -0.01, kf60);
        rangeindex_60 = rangeindex_60-1;
    else
        navstate_60 = InsMechBackward(laststate_60, lastimu_60, thisimu_60);
        navstate_60.pos(3) = height(imuindex-1-rr,2) ;
        kf60 = myInsPropagate_15state(navstate_60, thisimu_60, -0.01, kf60);
    end
    
    lastimu_60 = thisimu_60;
    thisimu_60 = imudata(imuindex-1-rr,:)';
    thisimu_60(2:4, 1) = (thisimu_60(2:4, 1) - imudt * cfg.initgyrbiasstd);
    thisimu_60(5:7, 1) = (thisimu_60(5:7, 1) - imudt * cfg.initaccbiasstd);
    laststate_60 = navstate_60;

    nav = zeros(11, 1);
    nav(2, 1) = laststate_60.time;
    nav(3:5, 1) = [laststate_60.pos(1) * param.R2D; laststate_60.pos(2) * param.R2D; laststate_60.pos(3)];
    nav(6:8, 1) = laststate_60.vel;
    nav(9:11, 1) = laststate_60.att * param.R2D;
    navdt(dt*100-rr,:) = nav;

    xb(dt*100-rr, :)  = kf60.x;
    Xbnav_seq(dt*100-rr, :) = [navstate_60.time; navstate_60.pos; navstate_60.vel; navstate_60.att];
    Pb(:, :, dt*100-rr) = kf60.P;
end

%%  全量双向平滑
idx_State = [2, 3, 5, 6, 7];
idx_P = [1, 2, 4, 5, 6];
for kk = 1:length(xb)
    kk_for = kk + dt*100*(floor((rangeindex-0.1)/dt)-1);
    Pf_full = P_seq(:, :, kk_for);
    Pb_full = Pb(:, :, kk);
 
    Pf_sub = Pf_full(idx_P, idx_P);
    Pb_sub = Pb_full(idx_P, idx_P);
    
    Val_A = Xnav_seq(kk_for, idx_State)';
    % Val_B = Xbnav_seq(kk, idx_State)' ;
    Val_B = Xbnav_seq(kk, idx_State)' - xb(kk, idx_P)';
    
    % Val_smooth = (Pf^-1 + Pb^-1)^-1 * (Pf^-1*Val_A + Pb^-1*Val_B);
 
    % 这里的逻辑是：为了避免 位置(1e-14) 和 速度(1e-1) 放在一起求逆导致矩阵奇异
    % 我们假设位置和速度的协方差耦合项较小，将其忽略，采用分块求逆。
    
    % --- 前向信息矩阵 If ---
    If = zeros(5, 5);
    % 1. 位置部分 (2x2)
    P_pos_f = Pf_sub(1:2, 1:2);
    if rcond(P_pos_f) < 1e-15
        If(1:2, 1:2) = zeros(2); % 如果奇异，说明方差无穷大，信息为0
    else
        If(1:2, 1:2) = inv(P_pos_f);
    end
    % 2. 速度部分 (3x3)
    P_vel_f = Pf_sub(3:5, 3:5);
    If(3:5, 3:5) = inv(P_vel_f); 
    
    % --- 后向信息矩阵 Ib ---
    Ib = zeros(5, 5);
    % 1. 位置部分
    P_pos_b = Pb_sub(1:2, 1:2);
    if rcond(P_pos_b) < 1e-15
        Ib(1:2, 1:2) = zeros(2); % 处理反向滤波初始阶段方差极大的情况
    else
        Ib(1:2, 1:2) = inv(P_pos_b);
    end
    % 2. 速度部分
    P_vel_b = Pb_sub(3:5, 3:5);
    % 增加一个保护：如果反向刚开始，方差可能是初始设的极大值，求逆应为0
    if rcond(P_vel_b) < 1e-15 
        Ib(3:5, 3:5) = zeros(3);
    else
        Ib(3:5, 3:5) = inv(P_vel_b);
    end
    P_smooth_sub = inv(If + Ib);
    Val_smooth = P_smooth_sub * (If * Val_A + Ib * Val_B);
    navbf(kk,:) = [0,Xnav_seq(kk_for, 1),Val_smooth(1:2)'* param.R2D,Xnav_seq(kk_for, 4),Val_smooth(3:5)',Xnav_seq(kk_for, 8:10)* param.R2D]  ;
    navbb(kk,:) = [0,Xnav_seq(kk_for, 1),Val_B(1:2)'* param.R2D,Xnav_seq(kk_for, 4),Val_B(3:5)',Xnav_seq(kk_for, 8:10)* param.R2D]  ;
end
fprintf(navbbfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navbb');
fprintf(navdtfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navdt');
fprintf(navbffp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navbf');


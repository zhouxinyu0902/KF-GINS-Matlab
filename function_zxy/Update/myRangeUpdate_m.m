function kf = myRangeUpdate_m(navstate, Rangedata, depthdata, kf)
    param = Param();
    bcn = Rangedata(4:6)';

    %% 1. 【核心修复】以载体当前纬度 navstate.pos(1) 计算地球半径，保证几何基准齐次
    [rm, rn] = getRmRn(navstate.pos(1) , param);
    h = navstate.pos(3);
    
    % 地理坐标差投影
    DR = diag([rm + h, (rn + h)*cos(navstate.pos(1)), -1]); 
    delta_pos = (DR * (navstate.pos - bcn))';
    HorizR = sqrt(sum(delta_pos(:,1:2).^2, 2));

    %% 2. 构造量测残差
    Z = [HorizR - Rangedata(3);
         navstate.pos(3) - depthdata(2)];
    kf.Z = Z;

    % 量测噪声
    vk = [kf.rangstd, kf.depthstd];
    R = diag(vk.^2);
    
    %% 3. 构建米制量测矩阵 H
    H = zeros(2, kf.RANK);
    H(1, 1) = delta_pos(1) / HorizR; % 北向位置偏导
    H(1, 2) = delta_pos(2) / HorizR; % 东向位置偏导
    H(2, 3) = -1;                     % 天向高度偏导
    
    kf.Zkk_1 = H * kf.x;

    %% 4. 经典的卡尔曼滤波更新
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - kf.Zkk_1);
    kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';

    %% 5. 反馈后残差二次计算
    pos_new = navstate.pos - kf.x(1:3);
    delta_pos_new = (DR * (pos_new - bcn))';
    SlantR = sqrt(sum(delta_pos_new(:,1:3).^2, 2));
    HorizR_new = sqrt(SlantR.^2 - delta_pos_new(:,3).^2);
    kf.Znew = HorizR_new - Rangedata(3);
end
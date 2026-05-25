function [kf, navstate] = myErrorFeedback_range_m(kf, navstate)
    
    %% 1. 【核心修复】将高度轴（第三维）的空间映射系数改为 1，切断天向轴正反馈自激
    DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
    DR_inv = inv(DR);
    
    % 米制误差通过 DR_inv 精准转换回绝对大地坐标(rad)，回拨主状态
    navstate.pos = navstate.pos - DR_inv * kf.x(1:3, 1);
    navstate.vel = navstate.vel - kf.x(4:6, 1);

    % 更新地理基础参数
    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);

    %% 2. 状态向量干净重置
    kf.x = zeros(kf.RANK, 1);
end
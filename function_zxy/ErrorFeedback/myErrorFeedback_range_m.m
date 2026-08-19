function [kf, navstate] = myErrorFeedback_range_m(kf, navstate)
    
    %% 1. 米制NED位置误差转换为名义大地坐标修正量
    DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
    DR_inv = diag([1 / DR(1, 1), 1 / DR(2, 2), -1]);
    
    % 第三维为下向误差x_D，故h_new = h - (-x_D) = h + x_D。
    navstate.pos = navstate.pos - DR_inv * kf.x(1:3, 1);
    navstate.vel = navstate.vel - kf.x(4:6, 1);

    % 与rad链保持相同反馈范围，避免两套状态定义比较时引入额外差异。
    navstate.gyrbias = navstate.gyrbias + kf.x(10:12, 1);
    navstate.accbias = navstate.accbias + kf.x(13:15, 1);

    % 更新地理基础参数
    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);

    %% 2. 状态向量干净重置
    kf.x = zeros(kf.RANK, 1);
end

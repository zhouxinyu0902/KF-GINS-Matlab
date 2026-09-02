function navstate = InsMechBackwardMEMS(laststate, lastimu, thisimu)
    % 反向运动学惯导编排
    % 输入:
    %   laststate - 时间上较晚的状态 (k时刻)
    %   lastimu   - 时间上较晚的IMU数据 (k时刻)
    %   thisimu   - 时间上较早的IMU数据 (k-1时刻)
    % 输出:
    %   navstate  - 时间上较早的状态 (k-1时刻)

    param = Param();
    
    % 复制较晚时刻的状态
    lastpos = laststate.pos;
    lastvel = laststate.vel;
    lastqbn = laststate.qbn;
    lastcbn = laststate.cbn;
    gravity = laststate.gravity;
    
    % 复制IMU数据并调整符号（关键反向处理）
    % 注意：时间间隔为负值（从较晚到较早）
    dt = thisimu(1, 1) - lastimu(1, 1);  % dt < 0
    
    % 角增量和速度增量取负（关键反向处理）
    last_dtheta = -lastimu(2:4, 1);
    last_dvel = -lastimu(5:7, 1);
    this_dtheta = -thisimu(2:4, 1);
    this_dvel = -thisimu(5:7, 1);
    
    %% 反向速度更新
    % 几何参数（使用较晚时刻的位置）
    rm = laststate.Rm;
    rn = laststate.Rn;
    wie_n = [param.WGS84_WIE * cos(lastpos(1)); 0; -param.WGS84_WIE * sin(lastpos(1))];
    wen_n = [lastvel(2) / (rn + lastpos(3)); 
            -lastvel(1) / (rm + lastpos(3)); 
            -lastvel(2) * tan(lastpos(1)) / (rn + lastpos(3))];
    
    % 旋转效应和双子样划桨效应（反向）
    temp1 = cross(this_dtheta, this_dvel) / 2;
    temp2 = cross(last_dtheta, this_dvel) / 12;
    temp3 = cross(last_dvel, this_dtheta) / 12;
    
    % b系比例积分项（反向）
    d_vfb = this_dvel + temp1 + temp2 + temp3;
    
    % n系比例积分项（反向）
    temp1 = (wie_n + wen_n) * dt / 2;  % dt < 0
    cnn = eye(3) - skew(temp1);
    d_vfn = cnn * lastcbn * d_vfb;
    
    % 重力/哥氏积分项（反向）
    gl = [0; 0; gravity];
    d_vgn = (gl - cross(2 * wie_n + wen_n, lastvel)) * dt;  % dt < 0
    
    % 中间时刻速度（反向）
    midvel = lastvel + (d_vfn + d_vgn) / 2;
    
    %% 反向位置更新
    % 外推中间时刻的位置（反向）
    temp = (wie_n + wen_n) * dt / 2;  % dt < 0
    qnn = rotvec2quat(temp);
    temp = [0; 0; -param.WGS84_WIE * dt / 2];  % dt < 0
    qee = rotvec2quat(temp);
    qne = bl2qne(lastpos(1), lastpos(2));
    qne = quatProd(quatProd(qee, qne), qnn);
    [midlat, midlon] = qne2bl(qne);
    midheight = lastpos(3) - midvel(3) * dt / 2;  % dt < 0
    midpos = [midlat; midlon; midheight];
    
    % 重新计算中间时刻的rmrn, wie_e, wen_n
    [rm, rn] = getRmRn(midpos(1), param);
    wie_n = [param.WGS84_WIE * cos(midpos(1)); 0; -param.WGS84_WIE * sin(midpos(1))];
    wen_n = [midvel(2) / (rn + midpos(3)); -midvel(1) / (rm + midpos(3)); -midvel(2) * tan(midpos(1)) / (rn + midpos(3))];
    
    % 重新计算n系下平均比力积分项（反向）
    temp = (wie_n + wen_n) * dt / 2;  % dt < 0
    cnn = eye(3) - skew(temp);
    d_vfn = cnn * lastcbn * d_vfb;
    
    % 重新计算重力、哥式积分项（反向）
    gravity = getGravity(midpos);
    gl = [0; 0; gravity];
    d_vgn = (gl - cross(2 * wie_n + wen_n, midvel)) * dt;  % dt < 0
    
    % 速度更新完成（反向）
    thisvel = lastvel + d_vfn + d_vgn;
    
    %% 反向位置更新（续）
    % 重新计算中间时刻的速度和位置（反向）
    midvel = (lastvel + thisvel) / 2;
    DRi = diag([1 / (rm + lastpos(3)), 1 / ((rn + lastpos(3)) * cos(lastpos(1))), -1]);
    midpos = lastpos + DRi * midvel * dt / 2;  % dt < 0
    
    % 重新计算中间时刻地理参数
    [rm, rn] = getRmRn(midpos(1), param);
    wie_n = [param.WGS84_WIE * cos(midpos(1)); 0; -param.WGS84_WIE * sin(midpos(1))];
    wen_n = [midvel(2) / (rn + midpos(3)); -midvel(1) / (rm + midpos(3)); -midvel(2) * tan(midpos(1)) / (rn + midpos(3))];
    
    % 重新计算k时刻到k-1时刻 n系旋转矢量（反向）
    temp = (wie_n + wen_n) * dt;  % dt < 0
    qnn = rotvec2quat(temp);
    
    % e系转动等效旋转矢量（反向）
    temp = [0; 0; -param.WGS84_WIE * dt];  % dt < 0
    qee = rotvec2quat(temp);
    
    % 位置更新完成（反向）
    qne = bl2qne(lastpos(1), lastpos(2));
    qne = quatProd(quatProd(qee, qne), qnn);
    [lat, lon] = qne2bl(qne);
    height = lastpos(3) - midvel(3) * dt;  % dt < 0
    thispos = [lat; lon; height];
    
    %% 反向姿态更新
    % 重新计算中间时刻的速度和位置（反向）
    midvel = (lastvel + thisvel) / 2;
    last_qne = bl2qne(lastpos(1), lastpos(2));
    this_qne = bl2qne(thispos(1), thispos(2));
    delta_qnn = quatProd(quatInv(this_qne), last_qne);
    temp = quat2rotvec(delta_qnn);
    qne_mid = quatProd(last_qne, rotvec2quat(temp / 2));
    [midlat, matlon] = qne2bl(qne_mid);
    midheight = (lastpos(3) + thispos(3)) / 2;
    midpos = [midlat; matlon; midheight];
    
    % 重新计算中间时刻地理参数
    [rm, rn] = getRmRn(midpos(1), param);
    wie_n = [param.WGS84_WIE * cos(midpos(1)); 0; -param.WGS84_WIE * sin(midpos(1))];
    wen_n = [midvel(2) / (rn + midpos(3)); 
            -midvel(1) / (rm + midpos(3)); 
            -midvel(2) * tan(midpos(1)) / (rn + midpos(3))];
    
    % 计算n系的旋转四元数 k时刻到k-1时刻变换（反向）
    temp = -(wie_n + wen_n) * dt;  % dt < 0
    qnn = rotvec2quat(temp);
    
    % 计算b系旋转四元数 补偿二阶圆锥误差（反向）
    temp = this_dtheta + cross(last_dtheta, this_dtheta) / 12;
    qbb = rotvec2quat(temp);
    
    % 姿态更新完成（反向）
    thisqbn = quatProd(quatProd(qnn, lastqbn), qbb);
    thiscbn = quat2dcm(thisqbn);
    thiseuler = dcm2euler(thiscbn);
    
    %% 更新新的导航状态（较早时刻）
    navstate.time = thisimu(1, 1);  % 较早时刻的时间
    navstate.pos = thispos;
    navstate.vel = thisvel;
    navstate.qbn = thisqbn;
    navstate.cbn = thiscbn;
    navstate.att = thiseuler;
    
    % 重新计算几何参数
    [navstate.Rm, navstate.Rn] = getRmRn(thispos(1), param);
    navstate.gravity = getGravity(thispos);
    
    % 复制IMU误差（不更新）
    navstate.gyrbias = laststate.gyrbias;
    navstate.accbias = laststate.accbias;
    navstate.gyrscale = laststate.gyrscale;
    navstate.accscale = laststate.accscale;
end
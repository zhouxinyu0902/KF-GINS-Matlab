function my_pos = calc_position_from_beacon(beacon_xy, range, theta_deg, heading_deg)
    % 输入:
    % beacon_xy:   [East, North] 信标已知坐标
    % range:       测得的距离 (米)
    % theta_deg:   测得的相对基线法线方位角 (-180~180)
    % heading_deg: 当前航向角 (北偏东)
    %
    % 输出:
    % my_pos:      [East, North] 计算出的航行器位置

    % 1. 还原相对于船首的角度 (法线指向右舷，所以+90度是法线，反之要转回去)
    % 几何逻辑: Angle_to_Bow = Angle_to_Normal + 90
    rel_bearing_to_bow = theta_deg + 90;

    % 2. 计算全局真方位角 (True Bearing: North=0, East=90)
    % 这一步算的是向量 V_me_to_beacon 的角度
    true_bearing_deg = heading_deg + rel_bearing_to_bow;
    
    % 转换为弧度用于三角函数计算
    true_bearing_rad = deg2rad(true_bearing_deg);

    % 3. 计算位移向量 (Delta East, Delta North)
    % sin对应东(X), cos对应北(Y)
    delta_E = range * sin(true_bearing_rad);
    delta_N = range * cos(true_bearing_rad);

    % 4. 反向解算位置
    % 我的位置 = 信标位置 - (我指向信标的向量)
    my_E = beacon_xy(1) - delta_E;
    my_N = beacon_xy(2) - delta_N;

    my_pos = [my_E, my_N];
end
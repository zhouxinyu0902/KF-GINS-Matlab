function theta_to_normal=pos2azimuth(dr_xy,beacon_xy,dr_heading_deg)
% 输入:
    % beacon_xy: [X, Y] 其中 X=East, Y=North
    % dr_xy:     [X, Y] 其中 X=East, Y=North
    % dr_heading_deg: 推算航向角 (度, 北偏东为正)
    %
    % 输出:
    % theta_to_normal: 相对于基线法线(航向)的夹角 (度, -180~180)

    
    % 1. 提取坐标 (明确映射关系，防止搞混)
    beacon_E = beacon_xy(1);
    beacon_N = beacon_xy(2);
    
    dr_E = dr_xy(1);
    dr_N = dr_xy(2);
    
    % 2. 计算差分
    delta_E = beacon_E - dr_E;
    delta_N = beacon_N - dr_N;
    
    % 3. 计算全局真方位角 (0度=正北, 90度=正东)
    % 注意: atan2(y, x) 在导航里对应 atan2(East, North)
    true_bearing = rad2deg(atan2(delta_E, delta_N)); 
    
    % 4. 减去航向角，得到相对船首的角度 (Relative Bearing)
    rel_bearing_bow = true_bearing - dr_heading_deg;
    
    % 5. 减去90度 (基线法线指向右舷)
    % 如果你的基线法线是指向左舷的，这里改为 +90
    raw_theta = rel_bearing_bow - 90;   
    
    % 6. 归一化到 -180 ~ 180
    theta_to_normal = mod(raw_theta + 180, 360) - 180;
end
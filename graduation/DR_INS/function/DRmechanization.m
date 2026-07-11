function dr = DRmechanization(dr, dvl_k, compass_k, depth_k)
%DRMECHANIZATION DVL + 罗盘 + 深度计航位推算
%
% 输入：
%   dvl_k     = [t, vb_x, vb_y, vb_z]
%   compass_k = [t, pitch, roll, yaw]，rad
%   depth_k   = [t, depth]，depth为NED中的D，向下为正
%
% 内部采用PSINS形式：vn=[VE;VN;VU]，pos=[lat;lon;h]

    tk = dvl_k(1);
    dt = tk - dr.time;
    if dt <= 0 || ~isfinite(dt)
        dt = dr.ts;
    end

    % 罗盘姿态。若DRfeedback估计了航向修正，则加到yaw上。
    att = compass_k(2:4)';
    if isfield(dr, 'yaw_corr')
        att(3) = att(3) + dr.yaw_corr;
    end

    % DVL体坐标系速度。若估计了DVL刻度因子，则除以dr.kod。
    vb = dvl_k(2:min(4,size(dvl_k,2)))';
    if length(vb) == 2
        vb = [vb; 0];
    end
    vb = vb / dr.kod;

    Cn_b = a2mat(att);
    vn = Cn_b * vb;

    % 当前深度约束。depth为D向下为正，h=-D。
    if numel(depth_k) >= 2 && ~isnan(depth_k(2))
        dr.pos(3) = -depth_k(2);
    end

    % 地球参数与水平位置递推。
    dr.eth = earth(dr.pos, vn);
    dSn = vn(1:2) * dt;      % [dE; dN]

    Mpv = [0, 1/dr.eth.RMh;
           1/dr.eth.clRNh, 0];

    dr.ds = Mpv * dSn;
    dr.pos(1:2) = dr.pos(1:2) + dr.ds;

    dr.time = tk;
    dr.ts = dt;
    dr.att = att;
    dr.vn = vn;
    dr.eth = earth(dr.pos, dr.vn);
    dr.avp = [dr.att; dr.vn; dr.pos];
end

function dr = DRInitialize(cfg, dvl0, compass0, depth0)
%DRINITIALIZE 初始化DR主状态
%
% dr.pos = [lat; lon; h]，rad rad m，高度向上为正
% dr.vn  = [VE; VN; VU]，m/s
% dr.att = [pitch; roll; yaw]，rad

    glvs;

    dr = struct();
    dr.time = dvl0(1);
    dr.ts = cfg.range_time_tolerance * 2;

    dr.pos = cfg.pos0(:);
    if numel(depth0) >= 2 && ~isnan(depth0(2))
        dr.pos(3) = -depth0(2);   % depth为D向下为正，pos(3)=h向上为正
    end

    dr.att = compass0(2:4)';
    dr.vn  = zeros(3,1);

    dr.kod = cfg.init_kod;
    dr.yaw_corr = 0;

    % 用初始DVL估计一次速度，方便初始化eth
    vb = dvl0(2:min(4,size(dvl0,2)))';
    if length(vb) == 2
        vb = [vb; 0];
    end
    Cn_b = a2mat(dr.att);
    dr.vn = Cn_b * (vb / dr.kod);

    dr.eth = earth(dr.pos, dr.vn);
    dr.ds = zeros(2,1);
    dr.avp = [dr.att; dr.vn; dr.pos];
end

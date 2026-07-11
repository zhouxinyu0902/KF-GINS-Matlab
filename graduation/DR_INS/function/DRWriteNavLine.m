function DRWriteNavLine(fp, id, dr, cfg)
%DRWRITENAVLINE 写导航结果
% 输出：id time lat_deg lon_deg h VE VN VU pitch_deg roll_deg yaw_deg

    if cfg.output_in_degree
        R2D = 180/pi;
        lat = dr.pos(1) * R2D;
        lon = dr.pos(2) * R2D;
        att = dr.att * R2D;
    else
        lat = dr.pos(1);
        lon = dr.pos(2);
        att = dr.att;
    end

    nav = [id; dr.time; lat; lon; dr.pos(3); dr.vn; att];
    fprintf(fp, '%8d %12.6f %15.10f %15.10f %12.5f %12.6f %12.6f %12.6f %12.6f %12.6f %12.6f\n', nav);
end

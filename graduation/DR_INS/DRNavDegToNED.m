function ned_txt = DRNavDegToNED(nav, origin)
%DRNAVDEGTONED 将导航输出转换为局部NED，便于和reference.txt比较
% nav格式：id time lat_deg lon_deg h VE VN VU pitch_deg roll_deg yaw_deg
% 输出：time N E D pitch roll yaw VE VN VU

    D2R = pi/180;
    t = nav(:,2);
    lat = nav(:,3) * D2R;
    lon = nav(:,4) * D2R;
    h   = nav(:,5);
    att = nav(:,9:11) * D2R;
    vel = nav(:,6:8);

    lla = [lat, lon, h];
    ned = lla2ned_local_DR(lla, origin);

    ned_txt = [t, ned, att, vel];
end

function ned = lla2ned_local_DR(lla, origin)
    lat = lla(:,1);
    lon = lla(:,2);
    h   = lla(:,3);

    lat0 = origin(1);
    lon0 = origin(2);
    h0   = origin(3);

    a = 6378137.0;
    f = 1 / 298.257223563;
    e2 = f * (2 - f);

    sin_lat0 = sin(lat0);
    RM = a * (1 - e2) / (1 - e2 * sin_lat0^2)^(3/2);
    RN = a / sqrt(1 - e2 * sin_lat0^2);

    dLat = lat - lat0;
    dLon = lon - lon0;
    dH   = h - h0;

    N = dLat * (RM + h0);
    E = dLon * (RN + h0) * cos(lat0);
    D = -dH;

    ned = [N, E, D];
end

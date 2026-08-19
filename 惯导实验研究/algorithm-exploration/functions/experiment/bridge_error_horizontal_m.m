function value_m = bridge_error_horizontal_m( ...
        bridge_error, position, position_error_unit)
%BRIDGE_ERROR_HORIZONTAL_M 将RTS桥接误差统一换算为水平米制模长。
% position为[纬度rad, 经度rad, 高度m]。position_error_unit可为：
%   "rad"：bridge_error(1:3)=[dLat(rad),dLon(rad),dH(m)]；
%   "m"  ：bridge_error(1:3)=[dN(m),dE(m),dD(m)]。

    arguments
        bridge_error (:, 1) double
        position (3, 1) double
        position_error_unit (1, 1) string
    end

    if numel(bridge_error) < 2
        error('桥接误差至少需要两个水平位置分量。');
    end
    switch lower(position_error_unit)
        case "m"
            north_m = bridge_error(1);
            east_m = bridge_error(2);
        case "rad"
            [rm, rn] = getRmRn(position(1), Param());
            north_m = bridge_error(1) * (rm + position(3));
            east_m = bridge_error(2) * (rn + position(3)) ...
                * cos(position(1));
        otherwise
            error('未知位置误差单位：%s。应使用 "rad" 或 "m"。', ...
                position_error_unit);
    end
    value_m = hypot(north_m, east_m);
end

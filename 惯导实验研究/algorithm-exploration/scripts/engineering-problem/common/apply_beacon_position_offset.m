function updated_range = apply_beacon_position_offset(range_data, offset_enu_m)
%APPLY_BEACON_POSITION_OFFSET 为测距数据第4至6列加入ENU潜标位置偏差。
    if size(range_data, 2) < 6 || numel(offset_enu_m) ~= 3
        error('range_data 至少应有6列，offset_enu_m 应为 [E,N,U]。');
    end
    updated_range = range_data;
    latitude = range_data(:, 4);
    longitude = range_data(:, 5);
    height = range_data(:, 6);
    a = 6378137.0;
    e2 = 6.69437999014e-3;
    denominator = sqrt(1-e2*sin(latitude).^2);
    rn = a./denominator;
    rm = a*(1-e2)./denominator.^3;
    longitude_scale = (rn+height).*cos(latitude);
    if any(abs(longitude_scale) < 1)
        error('信标纬度过于接近极点，无法稳定换算东向误差。');
    end
    updated_range(:, 4) = latitude + offset_enu_m(2)./(rm+height);
    updated_range(:, 5) = longitude + offset_enu_m(1)./longitude_scale;
    updated_range(:, 6) = height + offset_enu_m(3);
end

function [dx, dy] = compute_dx_dy(a, b, lat1, lat2, lon1, lon2)
    % 输入：
    %   a: 长半轴（米）
    %   b: 短半轴（米）
    %   lat1_deg, lat2_deg: 两点纬度（度）
    %   lon1_deg, lon2_deg: 两点经度（度）
    % 输出：
    %   dx: 东向距离（米）
    %   dy: 北向距离（米）

    % 计算纬度差和经度差
    delta_phi = lat2 - lat1;
    delta_lambda = lon2 - lon1;

    % 平均纬度
    avg_lat = (lat1 + lat2) / 2;

    % 计算第一偏心率 e²
    e2 = (a^2 - b^2) / a^2;

    % 子午线曲率半径 M
    M = a * (1 - e2) / (1 - e2 * sin(avg_lat)^2)^(3/2);

    % 卯酉圈曲率半径 N
    N = a / sqrt(1 - e2 * sin(avg_lat)^2);

    % 计算 dy（北向距离）
    dy = M * delta_phi;

    % 计算 dx（东向距离）
    dx = N * cos(avg_lat) * delta_lambda;
end
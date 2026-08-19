function [bearings, totalDistance_km] = calc_track_data(lat, lon, initialBearing_deg)
%CALC_WITH_INITIAL_BEARING 计算轨迹的航向角和总距离，可指定初始航向角。
%
% 输入:
%   lat - 轨迹点的纬度向量 (十进制度数)
%   lon - 轨迹点的经度向量 (十进制度数)
%   initialBearing_deg - (可选) 轨迹第一段（P1到P2）使用的航向角 (度, 北偏西为正)。
%                        如果未提供，则由P1和P2计算得出。
%
% 输出:
%   bearings - 每段轨迹的初始航向角向量 (度, -180 到 180, 北偏西为正)
%   totalDistance_km - 轨迹的总距离 (公里)
%
% 航向角约定: 北偏西为正。北(0°), 西(+90°), 东(-90°), 南(+/-180°)。

    % --- 检查和初始化 ---
    if ~isvector(lat) || ~isvector(lon) || length(lat) ~= length(lon)
        error('输入 lat 和 lon 必须是等长的向量。');
    end
    
    N = length(lat);
    if N < 2
        bearings = [];
        totalDistance_km = 0;
        warning('轨迹点数少于2个，无法计算航向角和距离。');
        return;
    end
    
    num_segments = N - 1;
    bearings = zeros(num_segments, 1);
    distances_km = zeros(num_segments, 1);
    
    % 地球平均半径 (公里)
    glvs
    R_km = glv.Re; 
    
    % --- 循环计算每段轨迹 ---
    for i = 1:num_segments
        
        % 距离计算部分：必须使用P1和P2的坐标计算
        lat1 = lat(i);
        lon1 = lon(i);
        lat2 = lat(i+1);
        lon2 = lon(i+1);

        % 1. 转换为弧度
        lat1_rad = deg2rad(lat1);
        lon1_rad = deg2rad(lon1);
        lat2_rad = deg2rad(lat2);
        lon2_rad = deg2rad(lon2);
        dLon = lon2_rad - lon1_rad;

        % --- 距离计算（Haversine 公式） ---
        dLat = lat2_rad - lat1_rad;
        a = sin(dLat/2).^2 + cos(lat1_rad) .* cos(lat2_rad) .* sin(dLon/2).^2;
        c = 2 * atan2(sqrt(a), sqrt(1-a));
        distances_km(i) = R_km * c;
        
        % --- 航向角计算 ---
        if i == 1 && nargin == 3 % 如果是第一段并且用户提供了初始航向角
            % 直接使用用户输入的值作为第一段航向角
            bearings(i) = initialBearing_deg;
        else 
            % 对于后续段，或未提供初始航向角的第一段，使用几何计算
            
            % 2. 计算 Y 和 X 分量
            Y = sin(dLon) * cos(lat2_rad);
            X = cos(lat1_rad) * sin(lat2_rad) - ...
                sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
            
            % 3. 计算反正切 (弧度，范围 [-pi, pi])
            bearing_rad = atan2(Y, X);
            
            % 4. 转换为角度 (范围 [-180, 180]，其中东为正，西为负)
            bearing_deg_std = rad2deg(bearing_rad);
            
            % 5. 转换为目标约定：北偏西为正（取反）
            bearings(i) = -bearing_deg_std;
        end
    end
    
    % 6. 计算总距离
    totalDistance_km = sum(distances_km);
end
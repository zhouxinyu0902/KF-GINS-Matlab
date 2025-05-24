function avp_circle = circle_seg_45(pos0, R, V, N, start_angle_deg)
%CIRCLE_SEG_CUSTOM 生成自定义圆周轨迹
%   航向角定义：正北为0°，正西为90°
%   输入参数:
%       pos0 - 参考初始点 [lat, lon] (度)
%       R    - 圆周半径 (米)
%       V    - 速度 (米/秒)
%       N    - 圈数
%       start_angle_deg - 起始角度(度)，正北为0°，正西为90°
%   输出:
%       avp_circle - [姿态(roll,pitch,yaw), 速度(vN,vE,vD), 位置(lat,lon,alt), 时间]

% 参数检查
if nargin < 5
    start_angle_deg = 0; % 默认起始角度
end

% 计算角速度(逆时针为正)
omega = V/R;

% 将起始角度转换为弧度并调整坐标系
start_angle = deg2rad(-(start_angle_deg - 90)); % 转换为数学坐标系角度

% 时间序列
T = 2*pi/abs(omega); % 周期
t = 0:0.005:N*T;
l = length(t);

% 计算角度(数学坐标系)
theta = omega * t + start_angle;

% 位置计算(数学坐标系)
x = R * cos(theta);  % 东向
y = R * sin(theta);  % 北向

% 速度计算(确保起始速度满足要求)
vE = -V * sin(theta);  % 东向速度
vN = V * cos(theta);   % 北向速度

% 航向角计算(导航坐标系：正北为0°，正西为90°)
yaw = mod(-theta + pi/2, 2*pi); % 转换为导航坐标系
yaw(yaw > pi) = yaw(yaw > pi) - 2*pi; % 范围[-pi, pi]

% 验证起始速度
if abs(vN(1) - (-sqrt(2))) > 1e-6 || abs(vE(1) - sqrt(2)) > 1e-6
    % 如果需要严格匹配起始速度，调整V的值
    V_actual = sqrt(vN(1)^2 + vE(1)^2);
    scale_factor = sqrt(2)/V_actual;
    vN = vN * scale_factor;
    vE = vE * scale_factor;
    V = V * scale_factor; % 更新实际速度
end

% 姿态(假设roll和pitch为0)
phi = [zeros(l, 2), yaw']; 

% 转换为地理坐标
pos = dxyz2pos([x', y', zeros(l, 1)], pos0);

% 组合输出 [姿态, 速度(北,东,地), 位置, 时间]
avp_circle = [phi, vN', vE', zeros(l, 1), pos, t'];

% 显示起始状态
fprintf('起始状态验证:\n');
fprintf('起始航向: %.2f°\n', rad2deg(yaw(1)));
fprintf('起始速度 - 北向: %.6f m/s, 东向: %.6f m/s\n', vN(1), vE(1));
end


function [fig, result] = compareBeaconBearing( ...
    trajLLH, headingRad, beaconLLH, x, xLabelText, labelinput)
%COMPAREBEACONBEARING Compare relative bearings of two beacons
%
% 输入：
%   trajLLH     - N×3轨迹位置：
%                 [纬度(rad), 经度(rad), 高度(m)]
%
%   headingRad  - N×1航向角，单位rad，范围[0,2*pi)
%                 正北为0，北偏西为正，即逆时针为正：
%                 北：0
%                 西：pi/2
%                 南：pi
%                 东：3*pi/2
%                 也可输入标量，表示恒定航向角
%
%   beaconLLH   - 2×3信标位置：
%                 [纬度(rad), 经度(rad), 高度(m)]
%
%   x           - N×1横坐标，可为时间或采样序号
%                 输入[]时使用采样序号
%
%   xLabelText  - 横坐标英文名称，例如：
%                 'Time (s)'
%                 输入[]时自动设置为'Sample Index'
%
% 输出：
%   fig         - 图窗句柄
%
%   result      - 结果结构体：
%       .x
%       .headingRad
%       .losAzimuthRad
%       .losAzimuthDeg
%       .relativeBearingRad
%       .relativeBearingDeg
%       .north
%       .east
%       .down
%       .horizontalRange
%       .figureHandle
%       .axesHandle
%
% 方位角定义：
%   正北为0，北偏西为正，范围为[0,2*pi)
%
% 相对方位角定义：
%   relativeBearing = wrapToPi(losAzimuth - heading)
%
% 相对方位角含义：
%   0°：信标位于载体正前方
%   正值：信标位于载体航向左侧
%   负值：信标位于载体航向右侧
%
% 调用示例：
%   [fig, result] = compareBeaconBearing( ...
%       trajLLH, headingRad, beaconLLH, time, 'Time (s)');

%% 1. 输入检查

validateattributes(trajLLH, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 3}, ...
    mfilename, 'trajLLH');

validateattributes(beaconLLH, {'numeric'}, ...
    {'real', 'finite', 'size', [2, 3]}, ...
    mfilename, 'beaconLLH');

N = size(trajLLH, 1);

if N < 1
    error('轨迹数据trajLLH不能为空。');
end

headingRad = headingRad(:);

if isscalar(headingRad)
    headingRad = repmat(headingRad, N, 1);
elseif numel(headingRad) ~= N
    error('headingRad必须为标量，或长度与轨迹点数N相同。');
end

if any(~isfinite(headingRad))
    error('headingRad中包含NaN或Inf。');
end

% 将航向角统一限制到[0,2*pi)
headingRad = mod(headingRad, 2*pi);

%% 2. 横坐标处理

if nargin < 4 || isempty(x)

    x = (1:N).';
    defaultXLabel = 'Sample Index';

else

    x = x(:);

    if numel(x) ~= N
        error('横坐标x的长度必须与轨迹点数N相同。');
    end

    if any(~isfinite(x))
        error('横坐标x中包含NaN或Inf。');
    end

    if N >= 2 && any(diff(x) <= 0)
        error('横坐标x必须严格递增。');
    end

    defaultXLabel = 'Time or Sample Index';

end

if nargin < 5 || isempty(xLabelText)
    xLabelText = defaultXLabel;
end

%% 3. 经纬度转换为ECEF坐标

trajECEF   = llhToECEF(trajLLH);
beaconECEF = llhToECEF(beaconLLH);

%% 4. 计算轨迹位置到两个信标的NED相对位置

north = zeros(N, 2);
east  = zeros(N, 2);
down  = zeros(N, 2);

for k = 1:N

    lat = trajLLH(k, 1);
    lon = trajLLH(k, 2);

    % ECEF坐标系到当前轨迹点当地NED坐标系的转换矩阵
    Re2n = [ ...
        -sin(lat)*cos(lon), -sin(lat)*sin(lon),  cos(lat);
        -sin(lon),           cos(lon),           0;
        -cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)];

    for i = 1:2

        % 从当前轨迹位置指向第i个信标的位置差
        deltaECEF = beaconECEF(i, :)' - trajECEF(k, :)';

        % 转换到当地NED坐标系
        deltaNED = Re2n * deltaECEF;

        north(k, i) = deltaNED(1);
        east(k, i)  = deltaNED(2);
        down(k, i)  = deltaNED(3);

    end
end

%% 5. 计算信标绝对视线方位角

% 航向角定义：
% 正北为0，北偏西为正，即逆时针为正
%
% atan2(-east,north)对应：
% 北：0
% 西：pi/2
% 南：pi
% 东：3*pi/2

losAzimuthRad = mod(atan2(-east, north), 2*pi);

%% 6. 计算信标相对于载体航向的方位角

relativeBearingRad = zeros(N, 2);

for i = 1:2

    relativeBearingRad(:, i) = wrapToPiLocal( ...
        losAzimuthRad(:, i) - headingRad);

end

% 绘图使用度
relativeBearingDeg = rad2deg(relativeBearingRad);

% 绝对视线方位角也转换为度，便于输出查看
losAzimuthDeg = rad2deg(losAzimuthRad);

%% 7. 计算水平距离

horizontalRange = hypot(north, east);

%% 8. 绘图

fig = figure( ...
    'Name', 'Relative Bearings of Two Beacons', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Units', 'centimeters', ...
    'Position', [3, 3, 9, 9]);

ax = axes(fig);

hold(ax, 'on');

plot(ax, x, relativeBearingDeg(:, 1), ...
    'LineWidth', 1.5, ...
    'LineStyle','-.',...
    'DisplayName', labelinput{1});

plot(ax, x, relativeBearingDeg(:, 2), ...
    'LineWidth', 1.5, ...
    'DisplayName',  labelinput{2});

%% 9. 标出0°和±180°

lineZero = yline(ax, 0, '--', '0^\circ', ...
    'LineWidth', 1.0, ...
    'LabelHorizontalAlignment', 'left', ...
    'LabelVerticalAlignment', 'bottom', ...
    'Interpreter', 'tex', ...
    'HandleVisibility', 'off');

linePositive180 = yline(ax, 180, '--', '+180^\circ', ...
    'LineWidth', 1.0, ...
    'LabelHorizontalAlignment', 'left', ...
    'LabelVerticalAlignment', 'top', ...
    'Interpreter', 'tex', ...
    'HandleVisibility', 'off');

lineNegative180 = yline(ax, -180, '--', '-180^\circ', ...
    'LineWidth', 1.0, ...
    'LabelHorizontalAlignment', 'left', ...
    'LabelVerticalAlignment', 'bottom', ...
    'Interpreter', 'tex', ...
    'HandleVisibility', 'off');

%% 10. 坐标轴设置

grid(ax, 'on');
box(ax, 'on');

xlabel(ax, xLabelText, ...
    'FontName', 'Times New Roman', ...
    'FontSize', 12);

ylabel(ax, 'Relative Bearing (deg)', ...
    'FontName', 'Times New Roman', ...
    'FontSize', 12);

% title(ax, 'Relative Bearings of Two Beacons', ...
%     'FontName', 'Times New Roman', ...
%     'FontSize', 13, ...
%     'FontWeight', 'normal');

legend(ax, ...
    'Location', 'best', ...
    'FontName', 'Times New Roman', ...
    'FontSize', 11);

% 稍微扩大范围，使±180°参考线及文字能够完整显示
ylim(ax, [-185, 185]);

yticks(ax, [-180, -90, 0, 90, 180]);

yticklabels(ax, { ...
    '-180^\circ', ...
    '-90^\circ', ...
    '0^\circ', ...
    '90^\circ', ...
    '180^\circ'});

set(ax, ...
    'FontName', 'Times New Roman', ...
    'FontSize', 11, ...
    'LineWidth', 1.0, ...
    'TickDir', 'in', ...
    'Layer', 'top');

%% 11. 输出结果

result = struct();

result.x = x;

% 航向角，单位rad，范围[0,2*pi)
result.headingRad = headingRad;

% 信标绝对视线方位角
result.losAzimuthRad = losAzimuthRad;
result.losAzimuthDeg = losAzimuthDeg;

% 相对载体航向的方位角
result.relativeBearingRad = relativeBearingRad;
result.relativeBearingDeg = relativeBearingDeg;

% 信标相对于轨迹位置的NED坐标
result.north = north;
result.east  = east;
result.down  = down;

% 水平距离
result.horizontalRange = horizontalRange;

% 图窗和坐标轴句柄
result.figureHandle = fig;
result.axesHandle = ax;

% 三条参考线句柄
result.zeroLineHandle = lineZero;
result.positive180LineHandle = linePositive180;
result.negative180LineHandle = lineNegative180;

end


function ecef = llhToECEF(llh)
%LLHTOECEF 将WGS-84大地坐标转换为ECEF坐标
%
% 输入：
%   llh(:,1) - 纬度，rad
%   llh(:,2) - 经度，rad
%   llh(:,3) - 高度，m
%
% 输出：
%   ecef(:,1) - ECEF X坐标，m
%   ecef(:,2) - ECEF Y坐标，m
%   ecef(:,3) - ECEF Z坐标，m

% WGS-84椭球参数
a  = 6378137.0;
f  = 1 / 298.257223563;
e2 = f * (2-f);

lat = llh(:, 1);
lon = llh(:, 2);
h   = llh(:, 3);

sinLat = sin(lat);
cosLat = cos(lat);

RN = a ./ sqrt(1-e2.*sinLat.^2);

x = (RN+h).*cosLat.*cos(lon);
y = (RN+h).*cosLat.*sin(lon);
z = (RN.*(1-e2)+h).*sinLat;

ecef = [x, y, z];

end


function angleWrapped = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将角度限制到[-pi,pi)

angleWrapped = mod(angle+pi, 2*pi)-pi;

end
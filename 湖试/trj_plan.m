clc; clear; close all;

%% ===================== 1. 固定三角形顶点 =====================
% 顶点 A、B、C（纬度，经度）
lat = [36.33112700;
       36.32855600;
       36.33091574];
lon = [120.73397900;
       120.73260800;
       120.72939277];

names = {'A','B','C'};

%% ===================== 2. 经纬度转局部平面坐标 =====================
R = 6371000; % 地球半径，m
lat0 = mean(lat);
lon0 = mean(lon);

kx = R * cosd(lat0) * pi / 180;
ky = R * pi / 180;

% 局部平面坐标（East, North）
x = (lon - lon0) * kx;
y = (lat - lat0) * ky;

triXY = [x, y];
centroidXY = mean(triXY, 1);

%% ===================== 3. 计算三条边长度 =====================
edgeIdx = [1 2; 2 3; 3 1];
edgeNames = {'AB','BC','CA'};
edgeDist = zeros(3,1);

for i = 1:3
    p1 = edgeIdx(i,1);
    p2 = edgeIdx(i,2);
    edgeDist(i) = haversine_m(lat(p1), lon(p1), lat(p2), lon(p2));
end

%% ===================== 4. 生成三种示例航迹 =====================
[routeX1, routeY1] = makeFigure8Route(triXY, centroidXY);
[routeX2, routeY2] = makeSerpentineRoute(triXY, centroidXY);
[routeX3, routeY3] = makeInnerTriangleRoute(triXY, centroidXY);

routeLat1 = lat0 + routeY1 / ky;
routeLon1 = lon0 + routeX1 / kx;

routeLat2 = lat0 + routeY2 / ky;
routeLon2 = lon0 + routeX2 / kx;

routeLat3 = lat0 + routeY3 / ky;
routeLon3 = lon0 + routeX3 / kx;

routeLen1 = calcRouteLength(routeLat1, routeLon1);
routeLen2 = calcRouteLength(routeLat2, routeLon2);
routeLen3 = calcRouteLength(routeLat3, routeLon3);

%% ===================== 5. 统一显示范围 =====================
allLat = [lat; routeLat1(:); routeLat2(:); routeLat3(:)];
allLon = [lon; routeLon1(:); routeLon2(:); routeLon3(:)];

latPad = 0.12 * (max(allLat) - min(allLat));
lonPad = 0.12 * (max(allLon) - min(allLon));

if latPad == 0, latPad = 0.0005; end
if lonPad == 0, lonPad = 0.0005; end

latLim = [min(allLat)-latPad, max(allLat)+latPad];
lonLim = [min(allLon)-lonPad, max(allLon)+lonPad];

%% ===================== 6. 三子图绘制 =====================
fig = figure('Color','w','Position',[60 80 1680 620]);

ax1 = geoaxes(fig, 'Position', [0.03 0.08 0.29 0.84]);
drawOneRouteMap(ax1, lat, lon, names, edgeIdx, edgeNames, edgeDist, ...
    routeLat1, routeLon1, latLim, lonLim, ...
    sprintf('8字形航迹  |  总长 %.1f m', routeLen1));

ax2 = geoaxes(fig, 'Position', [0.355 0.08 0.29 0.84]);
drawOneRouteMap(ax2, lat, lon, names, edgeIdx, edgeNames, edgeDist, ...
    routeLat2, routeLon2, latLim, lonLim, ...
    sprintf('往复弓字形航迹  |  总长 %.1f m', routeLen2));

ax3 = geoaxes(fig, 'Position', [0.68 0.08 0.29 0.84]);
drawOneRouteMap(ax3, lat, lon, names, edgeIdx, edgeNames, edgeDist, ...
    routeLat3, routeLon3, latLim, lonLim, ...
    sprintf('内缩三角形环绕航迹  |  总长 %.1f m', routeLen3));

sgtitle('固定三角形区域三种示例航迹', 'FontWeight', 'bold');

%% ===================== 7. 命令行输出 =====================
fprintf('========== 三角形顶点 ==========\n');
for i = 1:3
    fprintf('%s: %.8f, %.8f\n', names{i}, lat(i), lon(i));
end

fprintf('\n========== 三条边长度 ==========\n');
for i = 1:3
    fprintf('%s = %.2f m\n', edgeNames{i}, edgeDist(i));
end

fprintf('\n========== 三种示例航迹长度 ==========\n');
fprintf('8字形航迹       = %.2f m\n', routeLen1);
fprintf('往复弓字形航迹 = %.2f m\n', routeLen2);
fprintf('内缩三角形航迹 = %.2f m\n', routeLen3);

%% ===================== 8. 可选导出 =====================
% exportgraphics(fig, 'three_route_subplots.png', 'Resolution', 600);

%% ===================== 局部函数 =====================

function drawOneRouteMap(gx, lat, lon, names, edgeIdx, edgeNames, edgeDist, ...
    routeLat, routeLon, latLim, lonLim, titleStr)

    hold(gx, 'on');
    geobasemap(gx, 'satellite');

    % 三角形
    geoplot(gx, lat([1 2 3 1]), lon([1 2 3 1]), ...
        '-', 'LineWidth', 2.2, 'Color', [1 0.8 0.1]);

    % 顶点
    geoscatter(gx, lat, lon, 75, 'filled', ...
        'MarkerFaceColor', [1 0.2 0.1], ...
        'MarkerEdgeColor', 'w', 'LineWidth', 1.0);

    % 顶点标注
    for i = 1:3
        text(gx, lat(i), lon(i), ['  ', names{i}], ...
            'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold');
    end

    % 边长标注
    for i = 1:3
        p = edgeIdx(i,:);
        latMid = mean(lat(p));
        lonMid = mean(lon(p));
        text(gx, latMid, lonMid, sprintf('%s: %.1f m', edgeNames{i}, edgeDist(i)), ...
            'Color', 'k', 'FontSize', 9.5, 'FontWeight', 'bold', ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'center');
    end

    % 航迹
    geoplot(gx, routeLat, routeLon, ...
        '-', 'LineWidth', 2.8, 'Color', [0.10 0.75 1.00]);

    % 起点和终点
    geoscatter(gx, routeLat(1), routeLon(1), 65, 'filled', ...
        'MarkerFaceColor', [0 1 0], 'MarkerEdgeColor', 'k');
    text(gx, routeLat(1), routeLon(1), '  起点', ...
        'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');

    geoscatter(gx, routeLat(end), routeLon(end), 65, 'filled', ...
        'MarkerFaceColor', [1 0 1], 'MarkerEdgeColor', 'k');
    text(gx, routeLat(end), routeLon(end), '  终点', ...
        'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold');

    % 用加粗短线模拟方向感
    arrowStep = max(10, floor(numel(routeLat)/12));
    for k = 1:arrowStep:(numel(routeLat)-1)
        geoplot(gx, routeLat(k:k+1), routeLon(k:k+1), ...
            '-', 'LineWidth', 3.8, 'Color', [0 0.95 0.95]);
    end

    geolimits(gx, latLim, lonLim);
    title(gx, titleStr, 'FontWeight', 'bold');
end

function d = haversine_m(lat1, lon1, lat2, lon2)
    R = 6371000;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2).^2 + cosd(lat1).*cosd(lat2).*sin(dlon/2).^2;
    d = 2 * R * atan2(sqrt(a), sqrt(1-a));
end

function totalLen = calcRouteLength(routeLat, routeLon)
    totalLen = 0;
    for i = 1:(numel(routeLat)-1)
        totalLen = totalLen + haversine_m(routeLat(i), routeLon(i), ...
                                          routeLat(i+1), routeLon(i+1));
    end
end

function [rx, ry] = makeInnerTriangleRoute(triXY, centroidXY)
    scale = 0.72;
    inner = centroidXY + scale * (triXY - centroidXY);

    pts = [inner; inner(1,:)];
    nSeg = 50;
    rx = [];
    ry = [];
    for i = 1:3
        xs = linspace(pts(i,1), pts(i+1,1), nSeg);
        ys = linspace(pts(i,2), pts(i+1,2), nSeg);
        rx = [rx, xs(1:end-1)];
        ry = [ry, ys(1:end-1)];
    end
    rx = [rx, rx(1)];
    ry = [ry, ry(1)];
end

function [rx, ry] = makeFigure8Route(triXY, centroidXY)
    t = linspace(0, 2*pi, 280);
    x0 = sin(t);
    y0 = 0.55 * sin(2*t);

    scaleBase = min([norm(triXY(1,:) - triXY(2,:)), ...
                     norm(triXY(2,:) - triXY(3,:)), ...
                     norm(triXY(3,:) - triXY(1,:))]) / 2.6;

    rx = centroidXY(1) + scaleBase * x0;
    ry = centroidXY(2) + scaleBase * y0;

    [rx, ry] = shrinkRouteIntoTriangle(rx, ry, triXY, centroidXY);
end

function [rx, ry] = makeSerpentineRoute(triXY, centroidXY)
    safeScale = 0.72;
    inner = centroidXY + safeScale * (triXY - centroidXY);

    edges = [1 2; 2 3; 3 1];
    L = zeros(3,1);
    for i = 1:3
        p1 = inner(edges(i,1),:);
        p2 = inner(edges(i,2),:);
        L(i) = norm(p2 - p1);
    end
    [~, idx] = max(L);
    basePair = edges(idx,:);
    p1 = inner(basePair(1),:);
    p2 = inner(basePair(2),:);

    u = (p2 - p1);
    u = u / norm(u);
    v = [-u(2), u(1)];

    Q = [u(:), v(:)];
    P = (inner - centroidXY) / Q.';

    umin = min(P(:,1)); umax = max(P(:,1));
    vmin = min(P(:,2)); vmax = max(P(:,2));

    nLines = 4;
    vList = linspace(vmin*0.75, vmax*0.75, nLines);

    route = [];
    for i = 1:nLines
        vv = vList(i);
        uu = lineIntersectRangeWithTriangle(vv, P);

        if numel(uu) < 2
            continue;
        end
        ua = uu(1); ub = uu(end);

        nSeg = 40;
        if mod(i,2)==1
            us = linspace(ua, ub, nSeg);
        else
            us = linspace(ub, ua, nSeg);
        end
        vs = vv * ones(size(us));

        xy = [us(:), vs(:)] * Q.' + centroidXY;
        route = [route; xy]; %#ok<AGROW>

        if i < nLines
            vv2 = vList(i+1);
            uu2 = lineIntersectRangeWithTriangle(vv2, P);
            if numel(uu2) >= 2
                if mod(i,2)==1
                    uEnd = ub;
                    uNext = uu2(end);
                else
                    uEnd = ua;
                    uNext = uu2(1);
                end
                connN = 15;
                uc = linspace(uEnd, uNext, connN);
                vc = linspace(vv, vv2, connN);
                xyConn = [uc(:), vc(:)] * Q.' + centroidXY;
                route = [route; xyConn]; %#ok<AGROW>
            end
        end
    end

    rx = route(:,1).';
    ry = route(:,2).';
    [rx, ry] = shrinkRouteIntoTriangle(rx, ry, triXY, centroidXY);
end

function uu = lineIntersectRangeWithTriangle(vv, P)
    P2 = [P; P(1,:)];
    uu = [];
    for i = 1:3
        a = P2(i,:);
        b = P2(i+1,:);
        v1 = a(2); v2 = b(2);
        if (vv >= min(v1,v2)) && (vv <= max(v1,v2)) && abs(v2-v1) > 1e-10
            t = (vv - v1) / (v2 - v1);
            if t >= 0 && t <= 1
                u = a(1) + t*(b(1)-a(1));
                uu(end+1) = u; %#ok<AGROW>
            end
        elseif abs(vv-v1) < 1e-10 && abs(vv-v2) < 1e-10
            uu(end+1) = a(1); %#ok<AGROW>
            uu(end+1) = b(1); %#ok<AGROW>
        end
    end
    uu = unique(sort(uu));
end

function [rx2, ry2] = shrinkRouteIntoTriangle(rx, ry, triXY, centroidXY)
    route = [rx(:), ry(:)];
    scale = 1.0;
    for iter = 1:30
        test = centroidXY + scale * (route - centroidXY);
        in = inpolygon(test(:,1), test(:,2), triXY(:,1), triXY(:,2));
        if all(in)
            rx2 = test(:,1).';
            ry2 = test(:,2).';
            return;
        end
        scale = scale * 0.95;
    end
    test = centroidXY + scale * (route - centroidXY);
    rx2 = test(:,1).';
    ry2 = test(:,2).';
end
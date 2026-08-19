clear; clc; close all;

%% ================================
% 三个布放点
%% ================================

% 左侧
lonA = 120.667427;
latA = 36.506337;

% 上侧
lonB = 120.873409;
latB = 36.575984;

% 下侧
lonC = 120.844974;
latC = 36.397231;

%% ================================
% GeoMap
%% ================================
figure('Position',[100 80 950 850]);

gx = geoaxes;
hold(gx,'on');

geobasemap(gx,'streets');

% 三角形
latTri = [latA latB latC latA];
lonTri = [lonA lonB lonC lonA];

geoplot(gx,latTri,lonTri,...
    '-','LineWidth',2.5);

% 三个顶点
geoscatter(gx,latA,lonA,100,'filled');
geoscatter(gx,latB,lonB,100,'filled');
geoscatter(gx,latC,lonC,100,'filled');

% 标注
text(gx,latA,lonA,'  A',...
    'FontSize',13,'FontWeight','bold');

text(gx,latB,lonB,'  B',...
    'FontSize',13,'FontWeight','bold');

text(gx,latC,lonC,'  C',...
    'FontSize',13,'FontWeight','bold');

% 地图范围
geolimits(gx,...
    [36.35 36.63],...
    [120.60 121.00]);

title(gx,'20 km 等边三角形位置示意');

%% ================================
% 三边距离检查
%% ================================

dAB = geoDistKm(latA,lonA,latB,lonB);
dBC = geoDistKm(latB,lonB,latC,lonC);
dCA = geoDistKm(latC,lonC,latA,lonA);

fprintf('\n-------------------------\n');
fprintf('AB = %.3f km\n',dAB);
fprintf('BC = %.3f km\n',dBC);
fprintf('CA = %.3f km\n',dCA);
fprintf('-------------------------\n');

%% ================================
% 输出三个信标位置
%% ================================

hA = 0;
hB = 0;
hC = 0;

fprintf('\n========== 三个信标位置 ==========\n');
fprintf('Beacon 1: Lat = %.6f deg, Lon = %.6f deg, H = %.2f m\n', ...
    latA, lonA, hA);

fprintf('Beacon 2: Lat = %.6f deg, Lon = %.6f deg, H = %.2f m\n', ...
    latB, lonB, hB);

fprintf('Beacon 3: Lat = %.6f deg, Lon = %.6f deg, H = %.2f m\n', ...
    latC, lonC, hC);

fprintf('==================================\n');

%%
fprintf('\n========== 三个信标位置 ==========\n');
beacon = [deg2rad(latA), deg2rad(lonA), hA;deg2rad(latB), deg2rad(lonB), hB;deg2rad(latC), deg2rad(lonC), hC];
fprintf('Beacon 1: Lat = %.12f rad, Lon = %.12f rad, H = %.2f m\n', ...
    deg2rad(latA), deg2rad(lonA), hA);

fprintf('Beacon 2: Lat = %.12f rad, Lon = %.12f rad, H = %.2f m\n', ...
    deg2rad(latB), deg2rad(lonB), hB);

fprintf('Beacon 3: Lat = %.12f rad, Lon = %.12f rad, H = %.2f m\n', ...
    deg2rad(latC), deg2rad(lonC), hC);
%% ================================
% 经纬度距离函数
%% ================================
function d = geoDistKm(lat1,lon1,lat2,lon2)

    R = 6371.0088; % km

    phi1 = deg2rad(lat1);
    phi2 = deg2rad(lat2);

    dphi = deg2rad(lat2-lat1);
    dlambda = deg2rad(lon2-lon1);

    a = sin(dphi/2).^2 + ...
        cos(phi1).*cos(phi2).*sin(dlambda/2).^2;

    d = 2*R*atan2(sqrt(a),sqrt(1-a));
end
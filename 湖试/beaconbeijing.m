
clc; clear; close all;

%% 1. 初始参考点
lonRef = 116.49837934374865;
latRef = 39.988688850951995;

side = 20000;           % 等边三角形边长，m
radius = side/sqrt(3);  % 中心至顶点距离
azimuth = [0;120;240]; % 三个顶点的方位角

initialRange = 200;     % 初始显示中心±200m，可自由缩放

%% 2. 创建交互式地图
fig = figure('Color','w','Position',[120 80 1050 850], ...
    'Name','鼠标选择等边三角形中心');

gx = geoaxes(fig);
hold(gx,'on');
geobasemap(gx,'streets');

% 初始参考位置
geoscatter(gx,latRef,lonRef,80,'filled', ...
    'MarkerFaceColor','r','MarkerEdgeColor','w');

% 初始地图范围
setMapRange(gx,latRef,lonRef,initialRange);

title(gx,'滚轮缩放 | 左键拖动 | 右键选择中心');

% 保留geoaxes的默认缩放和平移交互
enableDefaultInteractivity(gx);

% 监听鼠标右键事件，不覆盖地图默认交互
listener = event.listener(fig,'WindowMousePress', ...
    @(src,evt) selectMapCenter(src,evt,gx));

%% 3. 交互选择中心
while isgraphics(fig)
    % 等待右键选取
    uiwait(fig);

    if ~isgraphics(fig)
        return;
    end

    if ~isappdata(fig,'SelectedCenter')
        continue;
    end

    center = getappdata(fig,'SelectedCenter');
    lat0 = center(1);
    lon0 = center(2);

    % 显示临时中心
    hTemp = geoscatter(gx,lat0,lon0,110,'filled', ...
        'MarkerFaceColor',[0 1 0], ...
        'MarkerEdgeColor','w');

    msg = sprintf(['当前选定中心：\n',...
        '纬度：%.10f\n经度：%.10f\n\n',...
        '是否确认该位置？'],lat0,lon0);

    choice = questdlg(msg,'确认三角形中心', ...
        '确认','重新选择','确认');

    if strcmp(choice,'确认')
        delete(hTemp);
        break;
    end

    delete(hTemp);
    rmappdata(fig,'SelectedCenter');
    title(gx,'继续缩放或移动地图，右键重新选择中心');
end

% 选点完成，关闭监听
delete(listener);

fprintf('\n========== 最终中心 ==========\n');
fprintf('纬度：%.10f\n',lat0);
fprintf('经度：%.10f\n',lon0);

%% 4. 计算20km等边三角形顶点
lat = zeros(3,1);
lon = zeros(3,1);

for i = 1:3
    [lat(i),lon(i)] = vincentyDirect( ...
        lat0,lon0,azimuth(i),radius);
end

%% 5. 绘制最终三角形
geoplot(gx,lat([1 2 3 1]),lon([1 2 3 1]), ...
    'y-','LineWidth',2.5);

geoscatter(gx,lat,lon,100,'filled', ...
    'MarkerFaceColor','r','MarkerEdgeColor','w');

geoscatter(gx,lat0,lon0,110,'filled', ...
    'MarkerFaceColor',[0 0.8 0.2], ...
    'MarkerEdgeColor','w');

names = {'A','B','C'};

for i = 1:3
    text(gx,lat(i),lon(i),['  ',names{i}], ...
        'Color','w','FontSize',12,'FontWeight','bold');
end

text(gx,lat0,lon0,'  Center', ...
    'Color','w','FontSize',12,'FontWeight','bold');

% 标注边长
edges = [1 2;2 3;3 1];

for i = 1:3
    p = edges(i,:);
    text(gx,mean(lat(p)),mean(lon(p)), ...
        sprintf('%.1f km',side/1000), ...
        'Color','k','FontSize',11, ...
        'FontWeight','bold', ...
        'BackgroundColor','w', ...
        'HorizontalAlignment','center');
end

% 自动切换至整个三角形范围
latPad = 0.15*(max(lat)-min(lat));
lonPad = 0.15*(max(lon)-min(lon));

geolimits(gx, ...
    [min(lat)-latPad,max(lat)+latPad], ...
    [min(lon)-lonPad,max(lon)+lonPad]);

title(gx,'20 km等边三角形及中心');

%% 6. 打印结果
fprintf('\n========== 三角形顶点 ==========\n');
fprintf('中心：%.10f, %.10f\n',lat0,lon0);

for i = 1:3
    fprintf('%s：%.10f, %.10f\n', ...
        names{i},lat(i),lon(i));
end

fprintf('\n设计边长：%.3f km\n',side/1000);

%% 7. 可选图片导出
% exportgraphics(fig,'triangle_20km.png','Resolution',600);

%% 局部函数
function selectMapCenter(fig,evt,gx)
    % 只响应右键单击
    if ~strcmp(fig.SelectionType,'alt')
        return;
    end

    % 确认点击发生在地图区域
    hitObj = evt.HitObject;
    if isempty(hitObj) || ~isgraphics(hitObj)
        return;
    end

    if ~isequal(hitObj,gx)
        parentAx = ancestor(hitObj, ...
            'matlab.graphics.axis.GeographicAxes');
        if ~isequal(parentAx,gx)
            return;
        end
    end

    % geoaxes的CurrentPoint返回[纬度 经度]
    pt = gx.CurrentPoint;
    lat = pt(1,1);
    lon = pt(1,2);

    if ~all(isfinite([lat,lon]))
        return;
    end

    setappdata(fig,'SelectedCenter',[lat,lon]);
    uiresume(fig);
end

function setMapRange(gx,lat0,lon0,halfRange)
    mLat = 111132.92-559.82*cosd(2*lat0)+ ...
        1.175*cosd(4*lat0);
    mLon = 111412.84*cosd(lat0)-93.5*cosd(3*lat0);

    dLat = halfRange/mLat;
    dLon = halfRange/mLon;

    geolimits(gx, ...
        [lat0-dLat,lat0+dLat], ...
        [lon0-dLon,lon0+dLon]);
end

function [lat2,lon2] = vincentyDirect(lat1,lon1,az,dist)
    a = 6378137;
    f = 1/298.257223563;
    b = a*(1-f);

    alpha1 = deg2rad(az);
    phi1 = deg2rad(lat1);
    U1 = atan((1-f)*tan(phi1));

    sinU1 = sin(U1);
    cosU1 = cos(U1);
    sinA1 = sin(alpha1);
    cosA1 = cos(alpha1);

    sigma1 = atan2(tan(U1),cosA1);
    sinAlpha = cosU1*sinA1;
    cosSqAlpha = 1-sinAlpha^2;
    uSq = cosSqAlpha*(a^2-b^2)/b^2;

    A = 1+uSq/16384*(4096+ ...
        uSq*(-768+uSq*(320-175*uSq)));
    B = uSq/1024*(256+ ...
        uSq*(-128+uSq*(74-47*uSq)));

    sigma = dist/(b*A);

    for k = 1:100
        sinSigma = sin(sigma);
        cosSigma = cos(sigma);
        cos2SigmaM = cos(2*sigma1+sigma);

        deltaSigma = B*sinSigma*( ...
            cos2SigmaM+B/4*( ...
            cosSigma*(-1+2*cos2SigmaM^2)- ...
            B/6*cos2SigmaM*(-3+4*sinSigma^2)* ...
            (-3+4*cos2SigmaM^2)));

        sigmaNew = dist/(b*A)+deltaSigma;

        if abs(sigmaNew-sigma)<1e-13
            sigma = sigmaNew;
            break;
        end
        sigma = sigmaNew;
    end

    tmp = sinU1*sinSigma-cosU1*cosSigma*cosA1;

    phi2 = atan2( ...
        sinU1*cosSigma+cosU1*sinSigma*cosA1, ...
        (1-f)*sqrt(sinAlpha^2+tmp^2));

    lambda = atan2( ...
        sinSigma*sinA1, ...
        cosU1*cosSigma-sinU1*sinSigma*cosA1);

    C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));

    L = lambda-(1-C)*f*sinAlpha*( ...
        sigma+C*sinSigma*( ...
        cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM^2)));

    lat2 = rad2deg(phi2);
    lon2 = lon1+rad2deg(L);
end

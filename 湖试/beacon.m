
clc; clear; close all;

% 已知两个顶点
a = [36.331127,36.328556]; % 纬度
b = [120.733979,120.732608]; % 经度

% 创建地图
figure('Color','w','Position',[200 150 950 750]);
gx = geoaxes;
geobasemap(gx,'satellite');
hold(gx,'on');

% 绘制A、B两点及连线
geoplot(gx,a,b,'y-','LineWidth',2);
geoscatter(gx,a,b,80,'r','filled');

text(gx,a(1),b(1),' A','Color','w','FontSize',12,'FontWeight','bold');
text(gx,a(2),b(2),' B','Color','w','FontSize',12,'FontWeight','bold');

% 设置显示范围，向西侧预留空间
geolimits(gx,[min(a)-0.001,max(a)+0.001], ...
    [min(b)-0.004,max(b)+0.001]);

title(gx,'鼠标左键点击第三个顶点 C');
drawnow;

% 鼠标选择第三个点
geoaxes(gx);
[latC,lonC] = ginput(1);
if isempty(latC)
    return;
end

% 三个顶点坐标
lat = [a(:);latC];
lon = [b(:);lonC];

% 绘制三角形
geoplot(gx,lat([1 2 3 1]),lon([1 2 3 1]), ...
    'y-','LineWidth',2);
geoscatter(gx,latC,lonC,100,'r','filled');

text(gx,latC,lonC,' C','Color','w', ...
    'FontSize',12,'FontWeight','bold');

% 计算三条边的地表距离（Haversine）
R = 6371000;
idx = [1 2;2 3;3 1];
lat1 = lat(idx(:,1));
lat2 = lat(idx(:,2));
lon1 = lon(idx(:,1));
lon2 = lon(idx(:,2));

h = sind((lat2-lat1)/2).^2 + ...
    cosd(lat1).*cosd(lat2).*sind((lon2-lon1)/2).^2;
d = 2*R*atan2(sqrt(h),sqrt(1-h));

% 标注三条边距离
edgeNames = {'AB','BC','CA'};
for i = 1:3
    p = idx(i,:);
    latMid = mean(lat(p));
    lonMid = mean(lon(p));
    text(gx,latMid,lonMid, ...
        sprintf('%s: %.2f m',edgeNames{i},d(i)), ...
        'Color','k','FontSize',11, ...
        'FontWeight','bold', ...
        'BackgroundColor','w', ...
        'HorizontalAlignment','center');
end

title(gx,'三角形顶点及边长');

% 输出坐标和距离
fprintf('\n第三个顶点 C:\n');
fprintf('纬度：%.8f\n',latC);
fprintf('经度：%.8f\n',lonC);
fprintf('\n三角形边长：\n');
fprintf('AB = %.2f m\n',d(1));
fprintf('BC = %.2f m\n',d(2));
fprintf('CA = %.2f m\n',d(3));

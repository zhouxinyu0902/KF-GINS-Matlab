clc

trj11=importdata('轨迹数据_20250815T075113.751Z.csv');
lon=trj11.data(:,2);
lat=trj11.data(:,3);
% webmap OpenStreetMap;
% wmline(lat, lon, 'Color', 'b', 'Width', 3);
% wmmarker(lat(1), lon(1), 'Color', 'g'); %起点标识绿色
% wmmarker(lat(end), lon(end), 'Color', 'r'); %终点标识红色
plot(lon,lat)
clear;
clc;
close all;

%% =========================================================
% 01_read_raw_data.m
% 功能：
%   1. 读取各设备原始数据
%   2. 不进行时间截取、插值和数据构造
%   3. 保存为统一 MAT 文件供后续处理
%% =========================================================

%% 路径配置：第二批 run-0818
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(script_dir);
addpath(topic_dir, '-begin');
paths = setup_all_real_data_preprocessing('run-0818');
rootDir = paths.raw;
%% =========================================================
% 1. 读取 830 数据
%% =========================================================

filename  = fullfile(rootDir, '024014_4752450.dat');
filename1 = fullfile(rootDir, '024014_4752450_GPCHCX.dat');
filename2 = fullfile(rootDir, '024014_4752450_GPGGA.dat');

tic
[IMU_DATA_830, GPCHCX_830, GPGGA_830] = ...
    read_mems_ins(filename, filename1, filename2);

fprintf('830 原始数据读取完成\n');
toc
disp('-----------------------------');


%% =========================================================
% 2. 读取原始 IMU 文件
%% =========================================================

imuFile = fullfile(rootDir, 'imu_raw_data.txt');

IMU_raw = importdata(imuFile);

% 统一转换成数值矩阵
if isstruct(IMU_raw)
    IMU_raw = IMU_raw.data;
end

fprintf('IMU 原始数据读取完成：%d 行\n', size(IMU_raw,1));


%% =========================================================
% 3. 读取测距数据
%% =========================================================

rangeFile = fullfile(rootDir, 'range_data.txt');

range_raw = importdata(rangeFile);

if isstruct(range_raw)
    range_raw = range_raw.data;
end

fprintf('测距原始数据读取完成：%d 行\n', size(range_raw,1));


%% =========================================================
% 4. 读取深度数据
%% =========================================================

depthFile = fullfile(rootDir, 'depth_data.txt');

depth_raw = importdata(depthFile);

if isstruct(depth_raw)
    depth_raw = depth_raw.data;
end

fprintf('深度原始数据读取完成：%d 行\n', size(depth_raw,1));


%% =========================================================
% 5. 读取 AUXA 数据
%% =========================================================

auxaFile = fullfile(rootDir, 'auxa_fields.txt');

fid = fopen(auxaFile, 'r');

if fid < 0
    error('无法打开 AUXA 文件：%s', auxaFile);
end

% 前10列 double，第11列十六进制状态字
C = textscan(fid, ...
    '%f %f %f %f %f %f %f %f %f %f %x', ...
    'CommentStyle', '#');

fclose(fid);

% 前10列
auxa_raw = [C{1:10}];

% 状态字单独保留
auxa_state = C{11};

fprintf('AUXA 原始数据读取完成：%d 行\n', size(auxa_raw,1));


%% =========================================================
% 6. 如有需要：120原始数据
%% =========================================================

%{
% 查找 EB 90 32 帧头
frameHeader = [235, 144, 32];

filename120_imu = 'STDIMU_0928_1706~1901.dat';

fid = fopen(filename120_imu, 'rb');
allData2 = fread(fid, inf, 'uint8');
fclose(fid);

headerPositions2 = find( ...
    allData2(1:end-2) == frameHeader(1) & ...
    allData2(2:end-1) == frameHeader(2) & ...
    allData2(3:end)   == frameHeader(3));

IMU_FUR_file = read_stdimu_120( ...
    filename120_imu, headerPositions2);

filename120_auxa = 'AUAX_0928_1706~1746(有乱码).dat';
AUAX_file = read_auax_120(filename120_auxa);
%}


%% =========================================================
% 7. 如有需要：430原始数据
%% =========================================================

%{
filename430  = '0928_1700~1903_430.dat';
filename4301 = '0928_1700~1903_430_SEL_GPCHCX.dat';
filename4302 = '0928_1700~1903_430_SEL_GPGGA.dat';

[IMU_DATA_430, GPCHCX_430, GPGGA_430] = ...
    read_mems_ins(filename430, filename4301, filename4302);
%}


%% =========================================================
% 8. 保存全部原始数据
%% =========================================================

saveFile = paths.raw_mat_file;

save(saveFile, ...
    'IMU_DATA_830', ...
    'GPCHCX_830', ...
    'GPGGA_830', ...
    'IMU_raw', ...
    'range_raw', ...
    'depth_raw', ...
    'auxa_raw', ...
    'auxa_state', ...
    '-v7.3');

fprintf('\n========================================\n');
fprintf('原始数据已统一保存：\n%s\n', saveFile);
fprintf('========================================\n');

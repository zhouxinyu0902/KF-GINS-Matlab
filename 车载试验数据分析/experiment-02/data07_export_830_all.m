clear;
clc;
close all;
%% ========================================================================
% data07_read00_830_raw.m
% 只读取830已经拆分好的三类数据：
%   binary
%   GPCHCX
%   GPGGA
% 暂时不做截取、坐标转换和比较
% ========================================================================
%% 0. 路径
raw_dir = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\raw';
temp_dir = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\temp';
if ~exist(temp_dir,'dir')
    mkdir(temp_dir);
end
%% 1. 文件配置
FILE_830 = {
    '043941_4752450_binary.dat','043941_4752450_GPCHCX.txt','043941_4752450_GPGGA.txt'
    '083942_4752450_binary.dat','083942_4752450_GPCHCX.txt','083942_4752450_GPGGA.txt'
    '110223_4752450_binary.dat','110223_4752450_GPCHCX.txt','110223_4752450_GPGGA.txt'
    '150224_4752450_binary.dat','150224_4752450_GPCHCX.txt','150224_4752450_GPGGA.txt'
};
GROUP_NAME = {
    '043941_4752450'
    '083942_4752450'
    '110223_4752450'
    '150224_4752450'
};
%% 2. 逐组读取
nFile = size(FILE_830,1);
Raw830 = struct([]);
fprintf('\n============================================================\n');
fprintf('830原始数据读取\n');
fprintf('============================================================\n');
for i = 1:nFile
    fprintf('\n---------------- %d/%d：%s ----------------\n',i,nFile,GROUP_NAME{i});
    file_binary = fullfile(raw_dir,FILE_830{i,1});
    file_gpchcx = fullfile(raw_dir,FILE_830{i,2});
    file_gpgga = fullfile(raw_dir,FILE_830{i,3});
    if ~exist(file_binary,'file')
        error('binary文件不存在：%s',file_binary);
    end
    if ~exist(file_gpchcx,'file')
        error('GPCHCX文件不存在：%s',file_gpchcx);
    end
    if ~exist(file_gpgga,'file')
        error('GPGGA文件不存在：%s',file_gpgga);
    end
    fprintf('binary：%s\n',FILE_830{i,1});
    fprintf('GPCHCX：%s\n',FILE_830{i,2});
    fprintf('GPGGA ：%s\n',FILE_830{i,3});
    tic;
    IMU_DATA = importdata(file_binary);
    GPCHCX = importdata(file_gpchcx);
    GPGGA = importdata(file_gpgga);
    read_time = toc;
    Raw830(i).name = GROUP_NAME{i};
    Raw830(i).file_binary = FILE_830{i,1};
    Raw830(i).file_gpchcx = FILE_830{i,2};
    Raw830(i).file_gpgga = FILE_830{i,3};
    Raw830(i).imu = IMU_DATA;
    Raw830(i).gpchcx = GPCHCX;
    Raw830(i).gpgga = GPGGA;
    Raw830(i).read_time = read_time;
    fprintf('读取耗时：%.2f s\n',read_time);
    fprintf('IMU    ：');
    print_data_size(IMU_DATA);
    fprintf('GPCHCX ：');
    print_data_size(GPCHCX);
    fprintf('GPGGA  ：');
    print_data_size(GPGGA);
    if isnumeric(IMU_DATA) && ~isempty(IMU_DATA) && size(IMU_DATA,2) >= 7
        t = IMU_DATA(:,7);
        t = t(isfinite(t));
        if ~isempty(t)
            fprintf('IMU第7列范围：%.6f ~ %.6f\n',min(t),max(t));
        end
    end
end
%% 3. 总体检查
fprintf('\n============================================================\n');
fprintf('830全部读取结果\n');
fprintf('============================================================\n');
for i = 1:nFile
    fprintf('\n%s\n',Raw830(i).name);
    fprintf('IMU    ：');
    print_data_size(Raw830(i).imu);
    fprintf('GPCHCX ：');
    print_data_size(Raw830(i).gpchcx);
    fprintf('GPGGA  ：');
    print_data_size(Raw830(i).gpgga);
    fprintf('耗时   ：%.2f s\n',Raw830(i).read_time);
end
%% 4. 保存
save_file = fullfile(temp_dir,'raw830_all.mat');
save(save_file,'Raw830','-v7.3');
fprintf('\n============================================================\n');
fprintf('830原始数据保存完成\n');
fprintf('%s\n',save_file);
fprintf('============================================================\n');
%% ========================================================================
% 局部函数
% ========================================================================
function print_data_size(data)
if isnumeric(data) || iscell(data) || isstring(data)
    fprintf('%d × %d\n',size(data,1),size(data,2));
elseif isstruct(data)
    fprintf('struct，字段：%s\n',strjoin(fieldnames(data),', '));
else
    fprintf('%s\n',class(data));
end
end
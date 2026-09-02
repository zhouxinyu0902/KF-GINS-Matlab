function [IMU_DATA_430,GPCHCX_430,GPGGA_430]=read_mems_ins(filename,filename1,filename2)
% tic

% % 读取文件
fid = fopen(filename, 'rb');
allData = fread(fid, inf, 'uint8');
fclose(fid);
% % 帧头前8字节: AA 44 12 1C 0C 01 00 A0
frameHeader = [170, 68, 18, 28, 12, 1, 0, 160]; 
headerPositions = find(allData(1:end-5) == frameHeader(1) & ...
    allData(2:end-4) == frameHeader(2) & ...
    allData(3:end-3) == frameHeader(3)& ...
    allData(4:end-2) == frameHeader(4)& ...
    allData(5:end-1) == frameHeader(5)& ...
    allData(6:end) == frameHeader(6));
IMU_DATA_430=read_rawimub_430_830(filename,headerPositions);

%% ----提取GPCHCX
GPCHCX_430 = read_gpchcx_430_830(filename1);
%% ----提取GPGGA
GPGGA_430=read_gpgga_430_830(filename2);



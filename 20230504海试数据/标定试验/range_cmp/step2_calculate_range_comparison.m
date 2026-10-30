%% USBL水平测距完整流程入口
% 本脚本仅负责依次运行拆分后的三个脚本，不再包含具体计算代码。
%
% 使用方法
%   1. 在下面设置runBeaconId；
%   2. 直接运行本脚本即可完成USBL测距、时延测距和精度评估；
%   3. 也可以分别单独运行三个子脚本。
%
% ID设置
%   runBeaconId = 93;  % 单独处理ID93
%   runBeaconId = [];  % 处理step1合并文件中的全部ID

clearvars;
clc;
close all;

runBeaconId = 90;
scriptDir = fileparts(mfilename('fullpath'));
%%
fprintf('\n开始Step 2A：USBL几何水平距离。\n');
run(fullfile(scriptDir, 'step2a_calculate_usbl_horizontal_range.m'));
%%
fprintf('\n开始Step 2B：传播时延水平距离。\n');
run(fullfile(scriptDir, 'step2b_calculate_delay_horizontal_range.m'));


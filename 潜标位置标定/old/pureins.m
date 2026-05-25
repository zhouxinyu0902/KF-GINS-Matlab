% 支持纯惯导
%% 定义全局参数
clear;
glvs
load('data.mat')
%% 定义参数+加载过程配置
param = Param();
cfg = ProcessConfig_pure();
%% 加载数据
% imudata
imudata = importdata(cfg.imupath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
heightdata = trj.avp(:,[10,9]);
%% 获取处理时间，调整时间
starttime = imustarttime;
endtime = imuendtime;
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
if cfg.endtime > endtime
    cfg.endtime = endtime;
end
% data in process interval
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
heightdata(:,2) = heightdata(:,2)+normrnd(0,0.4,size(heightdata(:,2)));
heightdata = heightdata(heightdata(:,1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:,1) <= cfg.endtime, :);
%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult-pureins-height.nav'];
navfp = fopen(navpath, 'wt');
%% 调试
disp("Start INS Processing!");
lastprecent = 0;
%% 初始化
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;

lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    %% only do propagation
    % INS mechanization
    navstate = InsMech(laststate, lastimu, thisimu);
    navstate.pos(3) = heightdata(imuindex,2);
    % error propagation
    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    %% save data
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    
    %% print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.20) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end
%%
fclose(navfp);
disp("PureIns Integration Processing Finished!");
%%
calc_error(navpath,cfg.truthpath)
plot_trj(cfg.truthpath, navpath)
%%
plot_result(navpath)
%%
%% 1. 数据导入与预处理
% 假设 navpath 指向你的 .nav 文件
re = importdata(navpath);
re = re(1:388000,:); % 截取前 1900s 的数据

t  = re(:, 2);       % 时间列
vn = re(:, 6);       % 北向速度 (Vn)
ve = re(:, 7);       % 东向速度 (Ve)

ts = 0.01;           % 采样步长 (s)
Fs = 1/ts;           % 采样频率 (Hz)
L  = length(t);      % 信号长度

% 2. 频谱分析计算 (FFT)
% 减去均值以消除直流分量，突出休拉振荡等周期性误差成分 [cite: 198]
Y_n = fft(vn - mean(vn)); 
Y_e = fft(ve - mean(ve));

% 计算单侧幅频特性 P1
P2_n = abs(Y_n/L);
P1_n = P2_n(1:floor(L/2)+1);
P1_n(2:end-1) = 2*P1_n(2:end-1);

P2_e = abs(Y_e/L);
P1_e = P2_e(1:floor(L/2)+1);
P1_e(2:end-1) = 2*P1_e(2:end-1);

% 转换为分贝 (dB) 单位，便于同时观察不同量级的运动与误差 [cite: 512, 692]
% 公式：dB = 20 * log10(幅值)
P1_n_db = 20 * log10(P1_n + 1e-6); 
P1_e_db = 20 * log10(P1_e + 1e-6);

f = Fs*(0:(floor(L/2)))/L; % 频率轴

% 3. 理论频率定义 [cite: 125, 454]
f1_schuler = 1 / (84.4 * 60); % 休拉频率 (约 1.97e-4 Hz) [cite: 125, 453]
f2_motion  = 3 / 360;         % 3 deg/s 对应的螺旋运动频率 (约 0.0083 Hz)

% 4. 可视化绘图 (对照文档图 4-12 风格 [cite: 501])
figure('Color', 'w', 'Name', 'SINS 速度频谱分析');

% --- 子图 1：全频段观察 (突出显示螺旋运动频率 f2) ---
subplot(2,1,1);
plot(f, P1_n_db, 'r', 'LineWidth', 1.2); hold on;
plot(f, P1_e_db, 'b', 'LineWidth', 1.2);
grid on;
xlim([0, 0.02]); % 限制 X 轴范围以清晰显示 0.0083Hz 峰值
title('速度信号幅频特性 (全频段)');
ylabel('幅值 (dB)'); xlabel('频率 (Hz)');
legend('Vn (北向)', 'Ve (东向)', 'Location', 'northeast');

% 标注螺旋运动频率 [cite: 454]
line([f2_motion, f2_motion], ylim, 'Color', [0.5 0.5 0.5], 'LineStyle', ':');
text(f2_motion, max(ylim)*0.9, sprintf('  f2 (运动) %.4f Hz', f2_motion), 'FontSize', 9);

% --- 子图 2：低频段细节 (观察休拉振荡与惯导漂移误差 f1) ---
subplot(2,1,2);
plot(f, P1_n_db, 'r', 'LineWidth', 1.5); hold on;
plot(f, P1_e_db, 'b', 'LineWidth', 1.5);
grid on;
xlim([0, 0.001]); % 重点放大极低频区域 [cite: 457]
title('低频段细节 (观察休拉振荡/惯导漂移误差)');
ylabel('幅值 (dB)'); xlabel('频率 (Hz)');

% 标注理论休拉频率线 [cite: 125, 453]
line([f1_schuler, f1_schuler], ylim, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
text(f1_schuler, max(ylim)*0.8, '  f1 (休拉频率)', 'Color', 'k', 'FontWeight', 'bold');

% 调整整体布局
set(findall(gcf,'-property','FontSize'),'FontSize',10.5);
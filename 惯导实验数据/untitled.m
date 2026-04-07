%% 水声通信机接收机带通滤波器幅频特性分析
% 基于表6.2实测数据进行曲线拟合与指标提取
opts = detectImportOptions('D:\Program Files\Tencent\WeChat\xwechat_files\wxid_fc8lhduydwqm22_e67a\msg\file\2026-03\Magnitude(dB)(1).csv');
data = readtable('D:\Program Files\Tencent\WeChat\xwechat_files\wxid_fc8lhduydwqm22_e67a\msg\file\2026-03\Magnitude(dB)(1).csv', opts);

% 1. 原始实测数据录入
% 频率 f (kHz) 
f = [3.3, 5.6, 8.4, 10, 10.6, 11.2, 11.9, 12.6, ...
     13.35, 14.15, 14.99, 15.88, 16.8, 17.8, 18.87, 20, ...
     21.18, 22.24, 23.77, 25.18, 26.68, 28.26, 29.94, 35.5];

% 幅度 Gain (dB) 
gain_db = [-67.7, -52.78, -39.32, -32, -29.8, -27.02, -24.01, -20.71, ...
           -17.05, -12.93, -8.31, -3.41, 0.44, 1.84, 2, 2, ...
           2, 1.84, 0.44, -3.41, -8.31, -12.98, -17.05, -27.02];

f = data.Frequency / 1000;   % 将 Hz 转换为 kHz
gain_db = data.Nominal;

% 2. 曲线插值平滑处理 (提高分析精度)
f_high_res = linspace(min(f), max(f), 5000);
% 使用 pchip 插值，保持波形的单调性，避免在截止频率点附近产生虚假振荡
gain_smooth = interp1(f, gain_db, f_high_res, 'pchip');

% 3. 关键性能指标提取
[max_gain, max_idx] = max(gain_smooth); % 寻找最大增益点
f_center = f_high_res(max_idx);         % 确定中心频率

% 计算 -3dB 截止频率 (相对于峰值增益下降 3dB)
target_val = max_gain - 3; 
% 查找左右截止频率点索引
idx_pass = find(gain_smooth >= target_val);
f_low = f_high_res(idx_pass(1));
f_high = f_high_res(idx_pass(end));
bw_3db = f_high - f_low;                % 计算带宽

% 4. 绘图展示
fig=myfigurestartup(6,4,'paper');
% figure('Color', 'w', 'Name', '滤波器幅频特性分析');
plot(f, gain_db, 'rs', 'MarkerFaceColor', 'r', 'DisplayName', '实测数据点'); hold on;
plot(f_high_res, gain_smooth, 'b-', 'LineWidth', 1.8, 'DisplayName', '拟合特性曲线');
yline(target_val, '--k', sprintf('-3dB 阈值 (%.1f dB)', target_val), ...
    'LabelHorizontalAlignment', 'right', 'DisplayName', '-3dB 判定线','FontName','TimesSimSun');
plot([f_low, f_high], [target_val, target_val], 'kx', 'MarkerSize', 10, 'LineWidth', 2,'DisplayName', '截止频率点');

grid on;
set(gca, 'FontSize', 10);
xlabel('频率 f (kHz)', 'FontSize', 12);
ylabel('幅度 Gain (dB)', 'FontSize', 12);
% title('接收机带通滤波器幅频特性实测与拟合分析', 'FontSize', 14);
legend('Location', 'northeast');
ylim([-70 30])
exportgraphics(fig, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-111.png'), 'Resolution', 600);
% 5. 命令行结果输出
fprintf('====================================\n');
fprintf('       滤波器幅频特性分析报告         \n');
fprintf('====================================\n');
fprintf('峰值增益 (Peak Gain):    %6.2f dB\n', max_gain);
fprintf('中心频率 (Center Freq):  %6.2f kHz\n', f_center);
fprintf('下截止频率 (f_L, -3dB): %6.2f kHz\n', f_low);
fprintf('上截止频率 (f_H, -3dB): %6.2f kHz\n', f_high);
fprintf('-3dB 带宽 (Bandwidth):   %6.2f kHz\n', bw_3db);
fprintf('====================================\n');
%%
% 1. 读取数据 (假设文件名为 data.csv)
% 如果你的文件名不同，请修改此处
opts = detectImportOptions('D:\Program Files\Tencent\WeChat\xwechat_files\wxid_fc8lhduydwqm22_e67a\msg\file\2026-03\Magnitude(dB)(1).csv');
data = readtable('D:\Program Files\Tencent\WeChat\xwechat_files\wxid_fc8lhduydwqm22_e67a\msg\file\2026-03\Magnitude(dB)(1).csv', opts);

% 2. 提取数据并转换为 kHz
freq_khz = data.Frequency / 1000;   % 将 Hz 转换为 kHz
gain_nom = data.Nominal;
gain_min = data.Min;
gain_max = data.Max;

% 3. 创建高分辨率绘图窗口 (保留您的自定义函数)
% 如果没有 myfigurestartup 函数，可替换为 figure('Color','w');
fig = myfigurestartup(4, 3, 'paper'); 
hold on; 
grid on; 
% set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.5); % 细化网格线

% 4. 绘制阴影误差带 (最小值到最大值范围)
fill([freq_khz; flipud(freq_khz)], [gain_min; flipud(gain_max)], ...
     [0.0 0.45 0.74], 'EdgeColor', 'none', 'FaceAlpha', 0.15, ...
     'HandleVisibility', 'off'); 

% 5. 绘制标称值主曲线 (深蓝色加粗)
p = plot(freq_khz, gain_nom,'-', 'Color', [0 0.4470 0.7410]);

% 6. 设置坐标轴范围和刻度
xlim([0 70]);              % 锁定横坐标为 0~60 kHz
xticks(0:10:70);           % 设置每 10kHz 一个大刻度

% 7. 图表润色与中文标注
% FontName 使用 'SimSun' (宋体) 或您提到的 'TimesSimSun'
xlabel('频率 (kHz)', 'FontSize', 12, 'FontName', 'TimesSimSun');
ylabel('幅度响应 (dB)', 'FontSize', 12, 'FontName', 'TimesSimSun');
% title('滤波器幅频特性曲线 (0-60 kHz)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'TimesSimSun');

% 8. 优化图例和整体外观
% legend('Location', 'northeast', 'Box', 'on', 'FontName', 'TimesSimSun');
set(gca, 'Box', 'on', 'LineWidth', 1.2, 'TickDir', 'in', 'FontSize', 10);

% 9. 可选：添加 -3dB 截止频率参考线
% line([0 60], [-3 -3], 'Color', 'r', 'LineStyle', '--', 'HandleVisibility', 'off');

hold off;
exportgraphics(fig, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-111.png'), 'Resolution', 600);
%%
% 滤波器参数设置
N_standard = 6;      % 巴特沃斯、切比雪夫、椭圆的阶数
N_bessel = 6;        % 贝塞尔滤波器的阶数 (贝塞尔在频域下降较慢，通常需要更高阶数才能在相同截止频率达到-3dB)
                     % 注意：为了模拟原图中所有线交于-3dB点，MATLAB的'besself'设计时就需要指定-3dB截止频率。
                     % 如果完全同阶，它们的滚降率（下降斜率）会不同。原图看起来更侧重于展示各自形态的对比。

Rp = 2;              % 通带纹波峰-峰值 (dB)，用于切比雪夫I型和椭圆
Rs = 60;             % 阻带衰减 (dB)，用于椭圆
Wc = 1;              % 归一化截止频率 (通带/阻带 边界)

% 模拟频率矢量 (从0到3倍截止频率)
F_max = 3;
w = linspace(0, F_max * Wc, 1000); % 用于贝塞尔模拟频率

% 1. 设计滤波器模型并计算频率响应

% --- 巴特沃斯 (Butterworth) ---
% 通带最平坦，但在过渡带下降较慢。
[b_but, a_but] = butter(N_standard, Wc, 's');
h_but = freqs(b_but, a_but, w);

% --- 切比雪夫 I 型 (Chebyshev Type I) ---
% 通带内有纹波，过渡带下降比巴特沃斯快。
[b_cheb1, a_cheb1] = cheby1(N_standard, Rp, Wc, 's');
h_cheb1 = freqs(b_cheb1, a_cheb1, w);

% --- 贝塞尔 (Bessel) ---
% 线性相位/恒定群时延。在频域，幅度响应下降非常缓慢，过渡带最宽。
% MATLAB 'besself' 设计出的滤波器在Wc处不一定精确交于-3dB，
% 它是基于相位特性设计的。如果要精确模拟图中的-3dB交点，
% 需要对'besselap'进行频率变换。以下是标准设计。
[b_bes, a_bes] = besself(N_bessel, Wc);
h_bes = freqs(b_bes, a_bes, w);

% --- 椭圆 (Elliptic) ---
% 通带和阻带内均有纹波，但在相同阶数下过渡带最窄（下降最快）。
[b_ell, a_ell] = ellip(N_standard, Rp, Rs, Wc, 's');
h_ell = freqs(b_ell, a_ell, w);


% 2. 转换幅度为分贝 (dB)
% 并处理可能出现的极小值导致dB计算问题
mag_but = 20*log10(abs(h_but) + eps);
mag_cheb1 = 20*log10(abs(h_cheb1) + eps);
mag_bes = 20*log10(abs(h_bes) + eps);
mag_ell = 20*log10(abs(h_ell) + eps);


% 3. 绘图
% figure('Position', [100, 100, 800, 600]); % 设置图形窗口大小
myfigurestartup(4,4,'prese');
hold on;

% 绘图：注意颜色与原图尽量保持一致，并增加新的椭圆滤波器
% 颜色代码：巴特沃斯-蓝，切比雪夫-橙，贝塞尔-红，椭圆-紫（新加）
p4 = plot(w/Wc, mag_ell, 'Color', [0.5 0 0.5], 'LineWidth', 2.5); % 椭圆：紫色
p1 = plot(w/Wc, mag_but, 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5); % 巴特沃斯：标准蓝
p2 = plot(w/Wc, mag_cheb1, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5); % 切比雪夫I：标准橙
p3 = plot(w/Wc, mag_bes, 'Color', [0.6350 0.0780 0.1840], 'LineWidth', 2.5); % 贝塞尔：标准红

% p1 = plot(w/Wc, mag_but, 'b', 'LineWidth', 2.5); % 巴特沃斯
% p2 = plot(w/Wc, mag_cheb1, 'Color', [1 0.5 0], 'LineWidth', 2.5); % 切比雪夫I (橙色)
% p3 = plot(w/Wc, mag_bes, 'r', 'LineWidth', 2.5); % 贝塞尔
% p4 = plot(w/Wc, mag_ell, 'Color', [0.5 0 0.5], 'LineWidth', 2.5); % 椭圆 (紫色)

grid on;

% 模拟原图布局和标注

% 轴标注和设置
ylabel('幅值 (dB)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('归一化频率 \omega/\omega_c', 'FontSize', 14, 'FontWeight', 'bold'); % 原图只写了频率，这里更科学一点
% xlabel('频率', 'FontSize', 14); % 如果你坚持要像原图一样，就用这个

% 设置轴范围：Y轴主要模拟-90dB到+10dB（以便看清纹波）
axis([0 F_max -95 15]);

% 设置刻度
set(gca, 'XTick', [0 1], 'XTickLabel', {'0', '\omega_c'}); % 模拟原图只标记0，可以不标wc，取决于你
% set(gca, 'XTick', [0]); % 模拟原图只标0
set(gca, 'YTick', [-90 -3 0 10]);

% 添加关键参考线：-3dB线和Wc垂直线
xline(1, '--k', 'LineWidth', 1.5);
yline(-3, '--k', 'LineWidth', 1.5);

% 添加截止频率点标记 (可选)
plot(1, -3, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

% 添加图例
% 注意：为了匹配原图风格，我们将图例放在右上角。
leg = legend([p1, p2, p3, p4], ...
    {'巴特沃斯', '切比雪夫 (I型)', '贝塞尔', '椭圆 (新)'}, ...
    'Location', 'northeast', 'FontSize', 12);
% set(leg, 'Box', 'on', 'EdgeColor', 'k');


% 调整字体和图形外观
% set(gca, 'FontSize', 12, 'LineWidth', 1.5);
title('常用模拟低通滤波器幅度特性对比', 'FontSize', 16);

% 添加原图风格的注释 (可选，例如标注不同区域)
% text(0.2, 5, '通带纹波区', 'Color', [0.85 0.325 0.098], 'FontWeight', 'bold');
% text(1.1, -50, '过渡带滚降对比', 'FontWeight', 'bold');

hold off;
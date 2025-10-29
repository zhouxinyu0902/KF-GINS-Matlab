function [sigma, tau, Err] = allan_variance_analysis(y, tau0, varargin)
% ALLAN_VARIANCE_ANALYSIS Allan方差分析函数
%
% 功能：计算并分析信号的Allan方差，用于评估惯性传感器的噪声特性
%
% 输入参数：
%   y     - 输入数据序列（行或列向量）
%   tau0  - 采样周期（秒）
%   varargin - 可选参数：
%     'plot'      - 绘制结果图形（默认：true）
%     'title'     - 图形标题
%     'units'     - 数据单位（默认：根据y自动判断）
%
% 输出参数：
%   sigma - Allan方差值（与输入y单位一致）
%   tau   - 取样时间序列（秒）
%   Err   - 百分比估计误差
%
% 示例：
%   % 生成测试数据
%   fs = 100; tau0 = 1/fs;
%   y = randn(100000,1) + 0.00001*(1:100000)';
%   [sigma, tau, Err] = allan_variance_analysis(y, tau0, 'title', '陀螺仪Allan方差分析');
%
%   % 实际数据应用
%   load('imu_data.mat');
%   [sigma, tau, Err] = allan_variance_analysis(gyro_x, 0.01, 'units', 'rad/s');
%
% 参考：Yan Gong-min, 2012-08-22

% 参数解析
p = inputParser;
addRequired(p, 'y', @(x) isnumeric(x) && (isvector(x)));
addRequired(p, 'tau0', @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'plot', true, @islogical);
addParameter(p, 'title', 'Allan Variance Analysis', @ischar);
addParameter(p, 'units', '', @ischar);
parse(p, y, tau0, varargin{:});

% 确保y为列向量
y = y(:);
N = length(y);

% 初始化变量
sigma = [];
tau = [];
Err = [];
k = 1;
NL = N;
y_current = y;

% Allan方差计算主循环
while true
    % 计算当前分组级别的Allan方差
    diff_sq = (y_current(2:NL) - y_current(1:NL-1)).^2;
    sigma(k,1) = sqrt(1/(2*(NL-1)) * sum(diff_sq));
    
    % 记录取样时间和估计误差
    tau(k,1) = 2^(k-1) * tau0;
    Err(k,1) = 1 / sqrt(2*(NL-1));
    
    % 准备下一轮计算
    NL = floor(NL/2);
    if NL < 3
        break;
    end
    
    % 数据分组平均（长度加倍）
    y_current = 0.5 * (y_current(1:2:2*NL) + y_current(2:2:2*NL));
    k = k + 1;
end

% 可选：绘制结果图形
if p.Results.plot
    plot_allan_results(y, tau0, tau, sigma, Err, p.Results.title, p.Results.units);
end

% 显示关键噪声参数
if p.Results.plot
    identify_noise_parameters(tau, sigma);
end

end

%% 子函数：绘制Allan方差结果
function plot_allan_results(y, tau0, tau, sigma, Err, plot_title, units)
    figure('Name', 'Allan Variance Analysis', 'NumberTitle', 'off', ...
           'Position', [100, 100, 800, 600]);
    
    % 原始数据时域图
    subplot(2,2,1);
    t = tau0 * (0:length(y)-1);
    plot(t, y, 'b', 'LineWidth', 0.5);
    grid on;
    xlabel('时间 t / s');
    ylabel('幅值');
    title('原始数据时域图');
    if ~isempty(units)
        ylabel(['幅值 / ' units]);
    end
    
    % Allan方差对数图
    subplot(2,2,2);
    loglog(tau, sigma, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    hold on;
    loglog(tau, sigma.*(1+Err), 'r--', 'LineWidth', 1);
    loglog(tau, sigma.*(1-Err), 'r--', 'LineWidth', 1);
    grid on;
    xlabel('相关时间 \tau / s');
    ylabel('\sigma_A(\tau)');
    title('Allan方差曲线');
    legend('Allan方差', '误差边界', 'Location', 'best');
    if ~isempty(units)
        ylabel(['\sigma_A(\tau) / ' units]);
    end
    
    % 误差边界相对值
    subplot(2,2,3);
    semilogx(tau, Err*100, 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('相关时间 \tau / s');
    ylabel('估计误差 (%)');
    title('估计误差百分比');
    
    % 数据统计信息
    subplot(2,2,4);
    text(0.1, 0.8, sprintf('数据长度: %d points', length(y)), 'FontSize', 10);
    text(0.1, 0.6, sprintf('采样频率: %.1f Hz', 1/tau0), 'FontSize', 10);
    text(0.1, 0.4, sprintf('持续时间: %.1f s', length(y)*tau0), 'FontSize', 10);
    text(0.1, 0.2, sprintf('计算点数: %d', length(tau)), 'FontSize', 10);
    axis off;
    title('数据信息');
    
    sgtitle(plot_title);
end

%% 子函数：识别噪声参数
function identify_nouse_parameters(tau, sigma)
    fprintf('\n=== Allan方差噪声参数识别 ===\n');
    
    % 角度随机游走（ARW）
    [arw_min, idx] = min(sigma);
    arw_tau = tau(idx);
    fprintf('角度随机游走(ARW): %.3e @ τ=%.3fs\n', arw_min, arw_tau);
    
    % 零偏不稳定性（BI）
    [bi_plateau, bi_idx] = find_plateau(tau, sigma);
    if ~isempty(bi_plateau)
        fprintf('零偏不稳定性(BI): %.3e @ τ=%.3fs\n', bi_plateau, tau(bi_idx));
    end
    
    % 速率随机游走（RRW）
    slope = diff(log(sigma)) ./ diff(log(tau));
    fprintf('斜率范围: %.2f ~ %.2f\n', min(slope), max(slope));
end

%% 子函数：寻找平台区域（用于识别零偏不稳定性）
function [plateau_value, plateau_idx] = find_plateau(tau, sigma)
    % 简化实现：寻找斜率接近0的区域
    log_tau = log(tau);
    log_sigma = log(sigma);
    slopes = diff(log_sigma) ./ diff(log_tau);
    
    plateau_regions = find(abs(slopes) < 0.1);
    if ~isempty(plateau_regions)
        plateau_idx = plateau_regions(1);
        plateau_value = sigma(plateau_idx);
    else
        plateau_value = [];
        plateau_idx = [];
    end
end
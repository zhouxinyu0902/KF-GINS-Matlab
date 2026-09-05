clear;
clc;
close all;

%% ================================================================
%  潜标发声点圆环约束误差敏感性分析
%
%  基本关系：
%
%       r = sqrt(L^2 - H^2)
%
%       H = D_release - D_beacon
%
%  其中：
%       L : Release -> Beacon 实际绷紧绳长
%       H : 垂向高度差
%       r : Beacon 相对于 Release 的水平距离
%
%  当 L、D_release、D_beacon 存在误差时，
%  水平半径 r 不再是固定值，而形成 [r_min, r_max] 圆环。
%
%  本脚本分析：
%   1. 名义圆环
%   2. 不同误差来源对圆环宽度的影响
%   3. 不同倾角下误差放大情况
%   4. 绳长误差 + 深度误差联合影响
%   5. Monte-Carlo 随机误差传播
%
%% ================================================================

%% 1. 基本参数

% ---------------- 绳索参数 ----------------
L0 = 2500;              % 名义绷紧绳长 / m
err_L = 1;              % 绳长最大误差 ± / m

% ---------------- 名义倾角 ----------------
% 相对于竖直方向
theta0 = 10;            % deg

% ---------------- Release ----------------
D_release0 = 4000;      % Release 名义深度 / m
err_D_release = 10;     % Release 深度误差 ± / m

% ---------------- Beacon ----------------
%
% 为了使 L0、theta0、深度关系严格自洽，
% 根据 L0 和 theta0 计算名义垂向高度差。
%
H0 = L0 * cosd(theta0);

D_beacon0 = D_release0 - H0;

err_D_beacon = 5;       % Beacon 深度误差 ± / m

%% 名义水平偏移

r0 = sqrt(L0^2 - H0^2);

% 也等价于
r0_check = L0 * sind(theta0);

fprintf('\n====================================================\n');
fprintf('名义状态\n');
fprintf('====================================================\n');

fprintf('绳长 L0             = %.3f m\n', L0);
fprintf('倾角 theta0         = %.3f deg\n', theta0);

fprintf('Release 深度        = %.3f m\n', D_release0);
fprintf('Beacon 深度         = %.3f m\n', D_beacon0);

fprintf('垂向高度差 H0       = %.3f m\n', H0);
fprintf('水平半径 r0         = %.3f m\n', r0);

fprintf('\n');


%% ================================================================
% 2. 最坏情况圆环范围
% ================================================================

L_min = L0 - err_L;
L_max = L0 + err_L;

% H = D_release - D_beacon
%
% 最大 H：
%   Release 偏深 + Beacon 偏浅
%
% 最小 H：
%   Release 偏浅 + Beacon 偏深
%

H_min = ...
    (D_release0 - err_D_release) ...
    - (D_beacon0 + err_D_beacon);

H_max = ...
    (D_release0 + err_D_release) ...
    - (D_beacon0 - err_D_beacon);

[r_min, r_max, feasible] = ...
    radius_bounds(L_min, L_max, H_min, H_max);

if ~feasible
    error('当前参数不存在满足绷紧绳长约束的可行解。');
end

ring_width = r_max - r_min;

fprintf('====================================================\n');
fprintf('全部误差共同作用：最坏情况圆环\n');
fprintf('====================================================\n');

fprintf('L 范围              = [%.3f, %.3f] m\n', ...
    L_min, L_max);

fprintf('H 范围              = [%.3f, %.3f] m\n', ...
    H_min, H_max);

fprintf('圆环内半径 r_min    = %.3f m\n', r_min);
fprintf('圆环外半径 r_max    = %.3f m\n', r_max);

fprintf('圆环径向宽度        = %.3f m\n', ring_width);

fprintf('相对于名义半径：\n');
fprintf('向内扩展             = %.3f m\n', r0-r_min);
fprintf('向外扩展             = %.3f m\n', r_max-r0);

fprintf('\n');


%% ================================================================
% 3. Figure 1：俯视圆环
% ================================================================

figure(1);
clf;

angle_plot = linspace(0, 2*pi, 500);

x0 = r0*cos(angle_plot);
y0 = r0*sin(angle_plot);

x_inner = r_min*cos(angle_plot);
y_inner = r_min*sin(angle_plot);

x_outer = r_max*cos(angle_plot);
y_outer = r_max*sin(angle_plot);

% 填充圆环
x_ring = [x_outer, fliplr(x_inner)];
y_ring = [y_outer, fliplr(y_inner)];

fill(x_ring, y_ring, [0.85 0.85 0.85], ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.5);

hold on;

plot(x_inner, y_inner, ...
    '--', ...
    'LineWidth', 1.5);

plot(x_outer, y_outer, ...
    '--', ...
    'LineWidth', 1.5);

plot(x0, y0, ...
    'LineWidth', 2);

plot(0, 0, ...
    'kp', ...
    'MarkerSize', 12, ...
    'MarkerFaceColor', 'k');

axis equal;
grid on;

xlabel('East / m');
ylabel('North / m');

title(sprintf( ...
    '发声点水平圆环约束：\\theta = %.1f^\\circ', ...
    theta0));

legend( ...
    '误差允许圆环', ...
    sprintf('内边界 %.1f m',r_min), ...
    sprintf('外边界 %.1f m',r_max), ...
    sprintf('名义半径 %.1f m',r0), ...
    'Release', ...
    'Location','best');


%% ================================================================
% 4. Figure 2：不同倾角下，各误差来源对圆环宽度的影响
% ================================================================

theta_list = linspace(1, 20, 191);

Ntheta = length(theta_list);

width_L     = nan(Ntheta,1);
width_DR    = nan(Ntheta,1);
width_DB    = nan(Ntheta,1);
width_depth = nan(Ntheta,1);
width_all   = nan(Ntheta,1);

r_nominal = nan(Ntheta,1);

for k = 1:Ntheta

    theta = theta_list(k);

    H_nom = L0*cosd(theta);

    r_nominal(k) = L0*sind(theta);

    %% ------------------------------------------------------------
    % A. 只有绳长误差
    % -------------------------------------------------------------

    [r1,r2,ok] = radius_bounds( ...
        L0-err_L, ...
        L0+err_L, ...
        H_nom, ...
        H_nom);

    if ok
        width_L(k) = r2-r1;
    end


    %% ------------------------------------------------------------
    % B. 只有 Release 深度误差
    % -------------------------------------------------------------

    [r1,r2,ok] = radius_bounds( ...
        L0, ...
        L0, ...
        H_nom-err_D_release, ...
        H_nom+err_D_release);

    if ok
        width_DR(k) = r2-r1;
    end


    %% ------------------------------------------------------------
    % C. 只有 Beacon 深度误差
    % -------------------------------------------------------------

    [r1,r2,ok] = radius_bounds( ...
        L0, ...
        L0, ...
        H_nom-err_D_beacon, ...
        H_nom+err_D_beacon);

    if ok
        width_DB(k) = r2-r1;
    end


    %% ------------------------------------------------------------
    % D. 两个深度误差共同作用
    % -------------------------------------------------------------

    err_H = err_D_release + err_D_beacon;

    [r1,r2,ok] = radius_bounds( ...
        L0, ...
        L0, ...
        H_nom-err_H, ...
        H_nom+err_H);

    if ok
        width_depth(k) = r2-r1;
    end


    %% ------------------------------------------------------------
    % E. 全部误差
    % -------------------------------------------------------------

    [r1,r2,ok] = radius_bounds( ...
        L0-err_L, ...
        L0+err_L, ...
        H_nom-err_H, ...
        H_nom+err_H);

    if ok
        width_all(k) = r2-r1;
    end

end


figure(2);
clf;

plot(theta_list, width_L, ...
    'LineWidth', 1.7);

hold on;

plot(theta_list, width_DR, ...
    'LineWidth', 1.7);

plot(theta_list, width_DB, ...
    'LineWidth', 1.7);

plot(theta_list, width_depth, ...
    'LineWidth', 1.7);

plot(theta_list, width_all, ...
    'k', ...
    'LineWidth', 2.5);

xline(theta0, '--');

grid on;

xlabel('绳索相对竖直方向倾角 / deg');
ylabel('圆环径向宽度 r_{max}-r_{min} / m');

title('不同误差来源对圆环宽度的影响');

legend( ...
    sprintf('绳长误差 \\pm%.1f m',err_L), ...
    sprintf('Release深度误差 \\pm%.1f m',err_D_release), ...
    sprintf('Beacon深度误差 \\pm%.1f m',err_D_beacon), ...
    '全部深度误差', ...
    '全部误差', ...
    sprintf('当前 %.1f^\\circ',theta0), ...
    'Location','best');


%% ================================================================
% 5. Figure 3：水平半径对垂向高度差误差的敏感度
%
% dr/dH = -H/r = -cot(theta)
%
% 倾角越小，越接近竖直，误差放大越严重。
% ================================================================

sensitivity_H = abs(cotd(theta_list));

figure(3);
clf;

plot(theta_list, sensitivity_H, ...
    'LineWidth', 2);

grid on;

xlabel('倾角 / deg');
ylabel('|dr/dH|  (m/m)');

title('垂向高度差误差到水平半径误差的放大倍数');

xline(theta0,'--');

yline(abs(cotd(theta0)),'--');

text(theta0+0.5, ...
     abs(cotd(theta0)), ...
     sprintf('10^\\circ: %.2f m/m', ...
     abs(cotd(theta0))));


%% ================================================================
% 6. Figure 4：绳长误差 + 总深度误差联合敏感性
% ================================================================

err_L_list = linspace(0,5,101);

err_H_list = linspace(0,30,121);

ring_width_map = nan( ...
    length(err_H_list), ...
    length(err_L_list));

for i = 1:length(err_H_list)

    eH = err_H_list(i);

    for j = 1:length(err_L_list)

        eL = err_L_list(j);

        [r1,r2,ok] = radius_bounds( ...
            L0-eL, ...
            L0+eL, ...
            H0-eH, ...
            H0+eH);

        if ok
            ring_width_map(i,j) = r2-r1;
        end

    end

end


figure(4);
clf;

imagesc( ...
    err_L_list, ...
    err_H_list, ...
    ring_width_map);

set(gca,'YDir','normal');

colorbar;

xlabel('绳长误差 ± / m');
ylabel('垂向高度差误差 ± / m');

title(sprintf( ...
    '圆环宽度 / m，名义倾角 %.1f^\\circ', ...
    theta0));

hold on;

plot(err_L, ...
     err_D_release+err_D_beacon, ...
     'rp', ...
     'MarkerSize', 12, ...
     'MarkerFaceColor','r');

text( ...
    err_L+0.1, ...
    err_D_release+err_D_beacon, ...
    ' 当前误差设置');


%% ================================================================
% 7. Monte-Carlo 随机误差传播
% ================================================================

rng(0);

Nmc = 100000;

% 假设下面误差均服从均匀分布：
%
% [-err, +err]
%
% 如果后面知道标准差，也可改成 randn 高斯误差。

dL = ...
    (2*rand(Nmc,1)-1)*err_L;

dDR = ...
    (2*rand(Nmc,1)-1)*err_D_release;

dDB = ...
    (2*rand(Nmc,1)-1)*err_D_beacon;


L_mc = L0 + dL;

DR_mc = D_release0 + dDR;

DB_mc = D_beacon0 + dDB;

H_mc = DR_mc - DB_mc;

valid = L_mc >= abs(H_mc);

r_mc = nan(Nmc,1);

r_mc(valid) = sqrt( ...
    L_mc(valid).^2 ...
    - H_mc(valid).^2);

r_valid = r_mc(valid);


%% 统计

mean_r = mean(r_valid);
std_r  = std(r_valid);

r_sorted = sort(r_valid);

P025 = simple_percentile(r_sorted, 2.5);
P50  = simple_percentile(r_sorted, 50);
P975 = simple_percentile(r_sorted, 97.5);


fprintf('====================================================\n');
fprintf('Monte-Carlo 结果\n');
fprintf('====================================================\n');

fprintf('有效样本比例        = %.2f %%\n', ...
    100*sum(valid)/Nmc);

fprintf('平均水平半径        = %.3f m\n',mean_r);
fprintf('水平半径标准差      = %.3f m\n',std_r);

fprintf('2.5%% 分位数          = %.3f m\n',P025);
fprintf('50%% 分位数           = %.3f m\n',P50);
fprintf('97.5%% 分位数         = %.3f m\n',P975);


figure(5);
clf;

histogram( ...
    r_valid, ...
    80, ...
    'Normalization','pdf');

hold on;

xline(r0, ...
    'k-', ...
    'LineWidth', 2);

xline(r_min, ...
    '--', ...
    'LineWidth', 1.5);

xline(r_max, ...
    '--', ...
    'LineWidth', 1.5);

xline(P025, ...
    ':', ...
    'LineWidth', 1.5);

xline(P975, ...
    ':', ...
    'LineWidth', 1.5);

grid on;

xlabel('水平半径 r / m');
ylabel('概率密度');

title('多误差共同作用下水平半径 Monte-Carlo 分布');

legend( ...
    'Monte-Carlo', ...
    '名义值', ...
    '最坏情况内边界', ...
    '最坏情况外边界', ...
    '2.5%', ...
    '97.5%', ...
    'Location','best');


%% ================================================================
% 8. 输出当前倾角下的一阶误差敏感度
% ================================================================

dr_dL = L0/r0;

dr_dH = -H0/r0;

fprintf('\n====================================================\n');
fprintf('当前 %.1f deg 状态的一阶误差敏感度\n',theta0);
fprintf('====================================================\n');

fprintf('|dr/dL| = %.3f m/m\n',abs(dr_dL));
fprintf('|dr/dH| = %.3f m/m\n',abs(dr_dH));

fprintf('\n含义：\n');

fprintf('绳长误差 1 m，大约引起 %.2f m 水平半径变化。\n', ...
    abs(dr_dL));

fprintf('垂向高度差误差 1 m，大约引起 %.2f m 水平半径变化。\n', ...
    abs(dr_dH));


%% ================================================================
% Local function：由 L/H 区间求半径上下界
% ================================================================
function [rmin,rmax,feasible] = ...
    radius_bounds(Lmin,Lmax,Hmin,Hmax)

    rmin = NaN;
    rmax = NaN;
    feasible = false;

    % 只讨论正垂向高度差
    Hmin = max(Hmin,0);

    if Lmax < Hmin
        return;
    end

    feasible = true;

    %% 最大半径
    %
    % r 随 L 增大而增大；
    % r 随 H 增大而减小。
    %

    rmax = sqrt(max( ...
        Lmax^2-Hmin^2,0));


    %% 最小半径
    %
    % 如果 L、H 两个区间存在重叠，
    % 可以存在 L = H，此时水平半径为 0。
    %

    if Hmax >= Lmin

        rmin = 0;

    else

        rmin = sqrt(max( ...
            Lmin^2-Hmax^2,0));

    end

end


%% ================================================================
% 简单百分位数，避免依赖 Statistics Toolbox
% ================================================================
function value = simple_percentile(sortedData,p)

    N = length(sortedData);

    if N == 0
        value = NaN;
        return;
    end

    index = 1 + (N-1)*p/100;

    i1 = floor(index);
    i2 = ceil(index);

    if i1 == i2
        value = sortedData(i1);
    else

        alpha = index-i1;

        value = ...
            (1-alpha)*sortedData(i1) ...
            + alpha*sortedData(i2);

    end

end
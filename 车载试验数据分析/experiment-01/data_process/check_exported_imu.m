%% =========================================================================
% 导出 IMU 数据 TXT 可视化检查器 (IMU Viewer)
% 功能：同框对比 120、430、830 的三轴陀螺仪和三轴加速度计
% 格式要求：[时间, Gyro_X, Gyro_Y, Gyro_Z, Acc_X, Acc_Y, Acc_Z]
% =========================================================================
clear;
close all;

% ==================== 1. 配置要检查的组别 ====================
target_round = 6; % 【修改这里】你想检查第几组的 IMU 数据？(如 1, 2, 7, 8)
for target_round = 8
    % 设定基础路径
    data_dir = sprintf('D:\\Github\\KF-GINS-Matlab\\data\\experiment-data\\experiment-01\\case-0%d\\input', target_round);
    fprintf('正在加载 Round %d 的数据进行检查...\n路径: %s\n', target_round, data_dir);

    % ==================== 2. 安全加载 TXT 数据 ====================
    imu_120 = []; imu_430 = []; imu_830 = [];

    if exist(fullfile(data_dir, 'IMU_120.txt'), 'file'), imu_120 = readmatrix(fullfile(data_dir, 'IMU_120.txt')); end
    if exist(fullfile(data_dir, 'imu_430.txt'), 'file'), imu_430 = readmatrix(fullfile(data_dir, 'imu_430.txt')); end
    if exist(fullfile(data_dir, 'imu_830.txt'), 'file'), imu_830 = readmatrix(fullfile(data_dir, 'imu_830.txt')); end

    if isempty(imu_120) && isempty(imu_430) && isempty(imu_830)
        error('未在该路径下找到任何 IMU 文件，请检查路径是否正确！');
    end

    % ==================== 3. 提取时间轴与单位转换 ====================
    % 已根据您的指示修正：第1列是时间，2:4列是陀螺仪，5:7列是加速度计
    % 将弧度制陀螺仪转为 deg/s 方便观察
    rad2deg = 180 / pi;

    % 120 数据提取
    if ~isempty(imu_120)
        t_120   = imu_120(:, 1);             % 第1列：时间
        gyr_120 = imu_120(:, 2:4) * rad2deg; % 第2-4列：角速度 (rad/s -> deg/s)
        acc_120 = imu_120(:, 5:7);           % 第5-7列：比力 (m/s^2)
    end

    % 430 数据提取
    if ~isempty(imu_430)
        t_430   = imu_430(:, 1);
        gyr_430 = imu_430(:, 2:4) * rad2deg;
        acc_430 = imu_430(:, 5:7);
    end

    % 830 数据提取
    if ~isempty(imu_830)
        t_830   = imu_830(:, 1);
        gyr_830 = imu_830(:, 2:4) * rad2deg;
        acc_830 = imu_830(:, 5:7);
    end

    % ==================== 4. 绘图：陀螺仪 (Gyroscope) ====================
    figure('Name', sprintf('Round %d - 陀螺仪角速度对比', target_round), 'Position', [100, 100, 1200, 800]);
    sgtitle(sprintf('Round %d - 三轴陀螺仪数据对比 (FRD 坐标系)', target_round), 'FontSize', 14, 'FontWeight', 'bold');

    axes_names = {'X轴 (Front / Roll)', 'Y轴 (Right / Pitch)', 'Z轴 (Down / Yaw)'};
    for i = 1:3
        subplot(3, 1, i); hold on; grid on;
        % 绘制顺序：先画底层的 430/830，最后画 120 作为高精度参考
        if ~isempty(imu_430), plot(t_430, gyr_430(:,i), 'r-', 'LineWidth', 0.5, 'DisplayName', '430'); end
        if ~isempty(imu_830), plot(t_830, gyr_830(:,i), 'b-', 'LineWidth', 0.5, 'DisplayName', '830'); end
        if ~isempty(imu_120), plot(t_120, gyr_120(:,i), 'k-', 'LineWidth', 1.0, 'DisplayName', '120 (Ref)'); end

        ylabel('角速度 (deg/s)');
        title(axes_names{i});
        if i == 1, legend('Location', 'best'); end
        if i == 3, xlabel('GPS Time (s)'); end
        % 限制Y轴范围以过滤极个别毛刺，看清主体趋势
        % ylim([-50, 50]);
    end

    % ==================== 5. 绘图：加速度计 (Accelerometer) ====================
    figure('Name', sprintf('Round %d - 加速度计比力对比', target_round), 'Position', [150, 150, 1200, 800]);
    sgtitle(sprintf('Round %d - 三轴加速度计数据对比 (FRD 坐标系)', target_round), 'FontSize', 14, 'FontWeight', 'bold');

    for i = 1:3
        subplot(3, 1, i); hold on; grid on;
        if ~isempty(imu_430), plot(t_430, acc_430(:,i), 'r-', 'LineWidth', 0.5, 'DisplayName', '430'); end
        if ~isempty(imu_830), plot(t_830, acc_830(:,i), 'b-', 'LineWidth', 0.5, 'DisplayName', '830'); end
        if ~isempty(imu_120), plot(t_120, acc_120(:,i), 'k-', 'LineWidth', 1.0, 'DisplayName', '120 (Ref)'); end

        ylabel('比力 (m/s^2)');
        title(axes_names{i});
        if i == 1, legend('Location', 'best'); end
        if i == 3, xlabel('GPS Time (s)'); end

        % FRD(前右下)坐标系中，静止时Z轴(Down)测量到的重力反作用力为约 -9.8 m/s^2
        % if i == 3
        %     ylim([-15, -5]);
        % else
        %     ylim([-5, 5]);
        % end
    end

    % ==================== 6. 数据时长与采样率简报 ====================
    fprintf('\n======================================================\n');
    fprintf(' 📊 IMU 数据加载完毕 (Round %d)\n', target_round);
    fprintf('======================================================\n');

    if ~isempty(imu_120)
        fs = round(1 / mean(diff(t_120)));
        fprintf('[120 IMU]  记录数: %7d 行 | 采样率: ~%3d Hz | 时长: %.1f s\n', length(t_120), fs, t_120(end)-t_120(1));
    end
    if ~isempty(imu_430)
        fs = round(1 / mean(diff(t_430)));
        fprintf('[430 IMU]  记录数: %7d 行 | 采样率: ~%3d Hz | 时长: %.1f s\n', length(t_430), fs, t_430(end)-t_430(1));
    end
    if ~isempty(imu_830)
        fs = round(1 / mean(diff(t_830)));
        fprintf('[830 IMU]  记录数: %7d 行 | 采样率: ~%3d Hz | 时长: %.1f s\n', length(t_830), fs, t_830(end)-t_830(1));
    end
    fprintf('======================================================\n');
    fprintf('💡 提示：可通过图窗顶部的放大镜工具查看局部的高频对齐情况。\n');
end
function GJB_10s(gyro_x, fs, smoothing_time)
% GJB_10s 基于GJB标准计算陀螺仪零偏稳定性
% 
% 输入参数：
%   gyro_x          - 陀螺仪X轴角速度数据序列 (rad/s)
%   fs              - 采样频率 (Hz)
%   smoothing_time  - 平滑时间 (s)
%                     光纤或MEMS陀螺：10
%                     激光陀螺：100
%
% 输出：
%   在命令行窗口显示零偏稳定性分析结果
%   生成趋势图和分布直方图
%
% 示例：
%   gyro_x = imu_120(:, 3);     % 读取陀螺仪X轴数据
%   fs = 100;                   % 采样频率 100 Hz
%   smoothing_time = 10;        % 平滑时间 10秒
%   GJB_10s(gyro_x, fs, smoothing_time);

% 计算每段平滑数据点数
points_per_segment = smoothing_time * fs;

% 计算总段数（舍弃末尾不完整数据段）
num_segments = floor(length(gyro_x) / points_per_segment);

% 数据分段并计算每段平均值
segment_averages = zeros(num_segments, 1);

for i = 1:num_segments
    start_index = (i - 1) * points_per_segment + 1;
    end_index = i * points_per_segment;
    current_segment = gyro_x(start_index:end_index);
    segment_averages(i) = mean(current_segment);
end

% 计算所有段平均值的标准差（零偏稳定性）
bias_stability = std(segment_averages);

% 显示分析结果
fprintf('零偏稳定性分析结果:\n');
fprintf('陀螺仪轴别: X轴\n');
fprintf('平滑时间: %d 秒\n', smoothing_time);
fprintf('数据总段数: %d 段\n', num_segments);
fprintf('零偏稳定性 (1σ): %.6e rad/s\n', bias_stability);
fprintf('零偏稳定性 (1σ): %.6f °/h\n', rad2deg(bias_stability) * 3600);

% 绘制趋势图
figure('Name', '陀螺仪零偏趋势', 'NumberTitle', 'off');
plot((1:num_segments) * smoothing_time / 3600, segment_averages);
xlabel('时间 (小时)');
ylabel('平均角速度 (rad/s)');
title(sprintf('X轴陀螺仪 %d 秒平均值的趋势', smoothing_time));
grid on;

% 绘制分布直方图
figure('Name', '零偏分布直方图', 'NumberTitle', 'off');
histogram(segment_averages, 20);
xlabel('平均角速度 (rad/s)');
ylabel('频次');
title(sprintf('X轴陀螺仪 %d 秒平均值分布', smoothing_time));
grid on;

end
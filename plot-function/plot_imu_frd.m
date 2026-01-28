function [imu_full, lost] = plot_imu_frd(imu, t0)
    global glv
    if isempty(glv), psinstypedef; end
    dps = glv.dps; g0 = glv.g0;

    % --- 1. 识别缺失区间 ---
    t_raw = imu(:, 1);
    dt = diff(t_raw);
    ts = median(dt); % 统计意义上的标称周期
    
    % 找到跨度过大的缝隙索引 (即：该索引对应的点与下一个点之间有断层)
    gap_idx = find(dt > 1.5 * ts);
    t_insert_record=[];
    if ~isempty(gap_idx)
        for i=1:length(gap_idx)
            % --- 2. 局部补齐 (不改变原始采样点的时刻) ---
            % 每个 gap 的结尾前一瞬间插入一个点，维持 'previous' 填充逻辑
            % 这样绘图时，线段会保持平直直到下一个真实点出现
            t_insert = t_raw(gap_idx(i) ) + 0.01 :0.01: t_raw(gap_idx(i) + 1) - 0.01;
            % data_insert =repmat(imu(gap_idx(i), 2:7),length(t_insert),1) ;      % 保持上一时刻的值
            data_insert =repmat([6e-7,0,-5e-7,0,0,-0.098],length(t_insert),1) ;
            % 拼接并重新排序
            imu = [imu; [t_insert', data_insert]];
            t_insert_record = [t_insert_record;t_insert'];
        end

        [~, sort_idx] = sort(imu(:, 1));
        imu_full = imu(sort_idx, :);
        
        % 记录补齐点的位置（用于红色高亮）
        % 逻辑：在新矩阵中，寻找刚才插入的那些时间点
        lost = find(ismember(imu_full(:, 1), t_insert_record));
    else
        imu_full = imu;
        lost = [];
    end

    % --- 3. 绘图准备 ---
    t_full = imu_full(:, 1);
    if nargin < 2, t0 = t_full(1); end
    t_plot = t_full - t0;
    
    % 计算率值 (注意：非等间隔时，ts 的取值会影响单位换算)
    % 这里仍采用中位数 ts 换算，因为 imu 数据通常是增量(dTheta/dVel)
    gyro = imu_full(:, 2:4) / ts;
    acc  = imu_full(:, 5:7) / ts;

    % --- 4. 绘图 ---
    myfig;
    titles = {'X (Front)', 'Y (Right)', 'Z (Down)'};
    for i = 1:3
        % Gyro
        subplot(3, 2, 2*i-1);
        plot(t_plot, gyro(:, i)/dps, 'b-'); hold on;
        if ~isempty(lost)
            % 只在断层处画一个红叉，表示这里发生了数据丢失
            plot(t_plot(lost), gyro(lost, i)/dps, 'rx', 'MarkerSize', 6, 'LineWidth', 1);
        end
        grid on; ylabel([titles{i}, ' (deg/s)']);
        if i==1, title('Gyroscope (FRD - Cross marks Gap)'); end
        
        % Acc
        subplot(3, 2, 2*i);
        plot(t_plot, acc(:, i)/g0, 'g-'); hold on;
        if ~isempty(lost)
            plot(t_plot(lost), acc(lost, i)/g0, 'rx', 'MarkerSize', 6);
        end
        grid on; ylabel([titles{i}, ' (g)']);
        if i==1, title('Accelerometer (FRD)'); end
    end
    linkaxes(findobj(gcf, 'Type', 'axes'), 'x');
end
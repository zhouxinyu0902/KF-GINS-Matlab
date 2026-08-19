function nav_matrix = perform_block_smoothing(state_buffer, kf_x, param, range_idx)
    % 输入说明：
    % state_buffer: 结构体数组，pos(1:2)为rad, pos(3)为m
    % kf_x: 误差状态量，kf_x(1:2)为rad, kf_x(3)为m (高度误差)
    % kf_x(4:6)为速度误差 (m/s)
    
    num_pts = length(state_buffer);
    
    % 提取当前估计的误差向量
    % 注意：根据你的描述，1:3 已经是弧度/米组合
    pos_err = kf_x(1:3); 
    vel_err = kf_x(4:6); 
    
    % 预分配输出矩阵 (11 行, num_pts 列)
    nav_matrix = zeros(11, num_pts);
    
    for j = 1:num_pts
        % 计算线性平滑比例系数
        scale = j / num_pts; 
        
        % 1. 线性分摊当前点的误差
        % Δx_j = (j/N) * ΔX_total
        curr_pos_err = scale * pos_err;
        curr_vel_err = scale * vel_err;

        % 2. 执行修正 (单位均为 rad 或 m)
        % 修正后的值 = 惯导推算值 - 估计的误差值
        curr_pos_rad = state_buffer(j, 2:4)' - curr_pos_err;
        curr_vel = state_buffer(j, 5:7)' - curr_vel_err;
        
        % 3. 准备输出数据 (单位转换：弧度 -> 度)
        
        pos_deg = [curr_pos_rad(1:2) * param.R2D; curr_pos_rad(3)];
        att_deg = state_buffer(j, 8:10) * param.R2D;
        
        % 4. 填充 11 列格式矩阵
        % 对应你的 fprintf: %2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f
        nav_matrix(:, j) = [ ...
            range_idx; ...                       % 1. %2d (当前更新序号)
            state_buffer(j,1); ...            % 2. %12.6f (时间戳)
            pos_deg(1); pos_deg(2); pos_deg(3); ... % 3-5. 纬(deg), 经(deg), 高(m)
            curr_vel(1); curr_vel(2); curr_vel(3); ... % 6-8. 北东地速 (m/s)
            att_deg(1); att_deg(2); att_deg(3) ...   % 9-11. 滚俯航 (deg)
        ];
    end
end
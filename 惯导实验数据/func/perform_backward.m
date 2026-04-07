function nav_matrix_bw = perform_backward(imu_block, height_block, navstate_end, kf_end, param, meas, range_idx)
    % 输入说明：
    % imu_block    : [N x 7] 这一整段 7 分钟的 IMU 数据
    % height_block : [N x 2] 这一整段 7 分钟的高度数据 [time, height]
    % navstate_end : 7 分钟正向跑完、刚做完 RangeUpdate 后的最优名义状态
    % kf_end       : 7 分钟正向跑完的最优卡尔曼状态 (包含 P 阵)
    
    num_pts = size(imu_block, 1);
    
    % 预分配输出矩阵 (11 行 x N 列)，极速填充
    nav_matrix_bw = zeros(11, num_pts);
    
    % --- 1. 初始化终点 (作为反向推算的起点) ---
    curr_state = navstate_end;
    curr_kf = kf_end;
    
    % 取出这段数据的最后一条 IMU，作为时间倒流的第一个“上一时刻”
    this_imu = imu_block(num_pts, :)';
    
    % 记录第一点 (物理时间上的最后一点)
    nav_matrix_bw(:, num_pts) = [ ...
        range_idx; ...
        curr_state.time; ...
        curr_state.pos(1) * param.R2D; curr_state.pos(2) * param.R2D; curr_state.pos(3); ...
        curr_state.vel(1); curr_state.vel(2); curr_state.vel(3); ...
        curr_state.att(1) * param.R2D; curr_state.att(2) * param.R2D; curr_state.att(3) * param.R2D ...
    ];

    % --- 2. 反向推算主循环 ---
    % 从倒数第 2 个点，一路退回到第 1 个点
    for k = num_pts-1 : -1 : 1
        % 游标更替 (时间倒流)
        last_imu = this_imu;
        this_imu = imu_block(k, :)'; 
        
        % 时间步长 (在协方差推算时，Q 阵是随绝对时间生长的，用正值)
        dt = 0.01; 
        
        % A. 反向机械编排 (从 k+1 状态倒推回 k 状态)
        % 【高危提醒】你的 InsMechBackward 必须处理好地球自转反向和速度反向的逻辑！
        prev_state = InsMechBackward(curr_state, last_imu, this_imu);
        
        % B. 强制高度约束 (直接替换高度，或写成卡尔曼量测更新)
        prev_state.pos(3) = height_block(k, 2);
        
        
        % C. 反向误差协方差推算
        % 既然是往回推，不确定性依然是在增加的，P 阵会变大
        curr_kf = myInsPropagate_15state(prev_state, this_imu, dt, curr_kf);
        
        % 如果推到了这段数据的起点 (k=1)，并且你需要进行边界量测更新，可以写在这里
        if k == 1 &&range_idx ~=2
            curr_kf = myRangeUpdate(prev_state, meas.range, meas.height, curr_kf);
            [curr_kf, prev_state] = myErrorFeedback_range(curr_kf, prev_state);
        end
        
        % D. 记录当前点
        curr_state = prev_state;
        
        nav_matrix_bw(:, k) = [ ...
            range_idx; ...
            curr_state.time; ...
            curr_state.pos(1) * param.R2D; curr_state.pos(2) * param.R2D; curr_state.pos(3); ...
            curr_state.vel(1); curr_state.vel(2); curr_state.vel(3); ...
            curr_state.att(1) * param.R2D; curr_state.att(2) * param.R2D; curr_state.att(3) * param.R2D ...
        ];
    end
end
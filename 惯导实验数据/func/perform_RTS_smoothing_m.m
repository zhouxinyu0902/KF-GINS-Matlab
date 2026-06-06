function [nav_matrix,bridge_error,rtsstate_buffer] = perform_RTS_smoothing_m(state_buffer, Pk_corr_buffer, Pk_pred_buffer, PHI_buffer, xk_final, param, range_idx)
    % 输入说明：
    % state_buffer   : [N x 10] 矩阵 [time, lat, lon, h, vn, ve, vd, roll, pitch, yaw]
    % Pk_corr_buffer : [N x 225] 矩阵 (高度更新后的 P_{k|k})
    % Pk_pred_buffer : [N x 225] 矩阵 (惯性推算后的 P_{k+1|k})
    % PHI_buffer     : [N x 225] 矩阵 (状态转移矩阵 Phi_{k+1,k})
    % xk_final       : [15 x 1] 向量 (7min 时刻 RangeUpdate 计算出的终点误差)
    
    num_pts = size(state_buffer, 1);
    rank = 15; % 状态维数
    
    % 1. 初始化平滑误差存储
    xs = zeros(num_pts, rank);
    rtsstate_buffer = zeros(size(state_buffer));
    % 2. 注入“火种”：7min 时刻的测距终点误差
    xs(num_pts, :) = xk_final'; 
    


    % --- 反向平滑阶段 ---
    % 注意：必须从倒数第二个点开始回溯，避免 k+1 越界
    % --- 反向平滑阶段 ---
    % 从倒数第二个点开始回溯
    for k = num_pts-1 : -1 : 1
        
        % 1. 提取时刻 k 的校正协方差 P_{k|k}
        Pk_corr = reshape(Pk_corr_buffer(k, :), rank, rank);
        
        % 2. 【核心修复】索引改为 k！
        % 因为正向记录时，第 k 行存的就是推算到 k+1 的 Pk_pred 和 Phi
        Pk_pred = reshape(Pk_pred_buffer(k, :), rank, rank);
        Phi_next = reshape(PHI_buffer(k, :), rank, rank);
        
        % % 3. 【数值稳定保护】强制保证预测协方差矩阵的对称性
        % % 消除几万次推算带来的浮点数不对称误差，防止除法 (求逆) 发散
        % Pk_pred = (Pk_pred + Pk_pred') / 2;
        % 
        % % 4. 计算平滑增益 Gk
        % % MATLAB 右除算子 '/' 会自动调用最优化的求解器，比 inv() 更稳定
        % Gk = (Pk_corr * Phi_next') / Pk_pred;
        
        % 3. 【数值稳定保护第一道防线】强制对称
        Pk_pred = (Pk_pred + Pk_pred') / 2;
        
        % 【数值稳定保护第二道防线】对角线正则化 (Tikhonov Regularization)
        % 注入极小的白噪声底，防止某些不可观测的维度方差彻底塌陷为 0
        % 1e-12 是一个经验安全阈值，既能拉升条件数，又不会破坏物理协方差
        Pk_pred = Pk_pred + eye(rank) * 1e-12;
        
        % 4. 计算平滑增益 Gk
        % 【数值稳定保护第三道防线】使用奇异值分解求伪逆 (pinv)
        % 放弃传统的右除 '/'，改用 pinv。pinv 会自动丢弃那些导致矩阵奇异的极小特征值
        Gk = Pk_corr * Phi_next' * pinv(Pk_pred);
        
        % 5. 平滑状态误差
        xs(k, :) = (Gk * xs(k+1, :)')';
        
        % 可选：增加防爆力机制（如果发现在极个别点依然很大，可以加上阈值截断）
        % if any(abs(xs(k, 1:3)) > 1000)
        %     warning('平滑误差异常放大在第 %d 步', k);
        % end
    end
    bridge_error = xs(1, :)';
    
    % --- 构造输出矩阵 (将平滑后的误差 xs 补偿回 state_buffer) ---
    nav_matrix = zeros(11, num_pts);
    for j = 1:num_pts
        % 提取当前点平滑出的误差
        curr_xs = xs(j, :)';
        
        % 修正推算状态 (名义值 - 误差值)
        DR = diag([6335439 + state_buffer(j, 4), (6378137 + state_buffer(j, 4))*cos(state_buffer(j, 2)), -1]);
        DR_inv = inv(DR);
        curr_pos_rad = state_buffer(j, 2:4)' -DR_inv* curr_xs(1:3);
        curr_vel = state_buffer(j, 5:7)' - curr_xs(4:6);
        % curr_att_rad = state_buffer(j, 8:10)' - curr_xs(7:9);
        curr_att_rad = state_buffer(j, 8:10)' - zeros(3,1);
        % 单位转换 (rad -> deg)
        pos_deg = [curr_pos_rad(1:2) * param.R2D; curr_pos_rad(3)];
        vel_ned = curr_vel;
        att_deg = curr_att_rad * param.R2D;
        
        rtsstate_buffer(j,:) =[state_buffer(j, 1),curr_pos_rad',...
            curr_vel',curr_att_rad'] ; 

        % 填充 11 列格式
        nav_matrix(:, j) = [ ...
            range_idx; ...                       % 1. 块序号
            state_buffer(j, 1); ...              % 2. 时间
            pos_deg(1); pos_deg(2); pos_deg(3); ... % 3-5. 经纬高
            vel_ned(1); vel_ned(2); vel_ned(3); ... % 6-8. 速度
            att_deg(1); att_deg(2); att_deg(3)   ... % 9-11. 姿态
        ];
    end

end
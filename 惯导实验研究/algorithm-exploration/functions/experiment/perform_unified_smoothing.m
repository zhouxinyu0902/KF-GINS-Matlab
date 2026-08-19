function [nav_matrix, bridge_error, smoothed_state_buffer] = perform_unified_smoothing(...
    state_buffer, xk_final, param, range_idx, smooth_method, pos_unit, ...
    Pk_corr_buffer, Pk_pred_buffer, PHI_buffer)
% PERFORM_UNIFIED_SMOOTHING 统一的导航平滑处理函数
% 
% 输入参数:
%   state_buffer   : [N x 10] 推算状态缓存 [time, lat, lon, h, vn, ve, vd, roll, pitch, yaw]
%   xk_final       : [15 x 1] 测量更新计算出的终点误差状态
%   param          : 导航参数结构体 (包含 R2D 等)
%   range_idx      : 当前测距区间的序号
%   smooth_method  : 字符串, 'RTS' 或 'Linear'
%   pos_unit       : 字符串, 'm' (误差前三位为北东地米制) 或 'rad' (误差前三位为 弧度/弧度/米)
%   Pk_corr_buffer : [N x 225] (仅RTS需要，线性可传 [])
%   Pk_pred_buffer : [N x 225] (仅RTS需要，线性可传 [])
%   PHI_buffer     : [N x 225] (仅RTS需要，线性可传 [])
%
% 输出参数:
%   nav_matrix            : [11 x N] 用于 fprintf 写入文件的标准格式矩阵
%   bridge_error          : [15 x 1] 传递给上一个数据块的起始桥接误差
%   smoothed_state_buffer : [N x 10] 平滑修正后的状态缓存 (结构同 state_buffer)

    num_pts = size(state_buffer, 1);
    rank = length(xk_final); % 通常为 15 维
    
    % =====================================================================
    % 第一步：生成随时间变化的误差序列 xs (N x 15)
    % =====================================================================
    xs = zeros(num_pts, rank);
    
    if strcmpi(smooth_method, 'RTS')
        % --- RTS 反向平滑阶段 ---
        xs(num_pts, :) = xk_final'; 
        for k = num_pts-1 : -1 : 1
            Pk_corr = reshape(Pk_corr_buffer(k, :), rank, rank);
            Pk_pred = reshape(Pk_pred_buffer(k, :), rank, rank);
            Phi_next = reshape(PHI_buffer(k, :), rank, rank);
            
            % 数值稳定保护防线
            Pk_pred = (Pk_pred + Pk_pred') / 2;
            Pk_pred = Pk_pred + eye(rank) * 1e-12; % 极小正则化防塌陷
            
            % 计算平滑增益 Gk
            Gk = Pk_corr * Phi_next' * pinv(Pk_pred);
            
            % 平滑状态误差
            xs(k, :) = (Gk * xs(k+1, :)')';
        end
        
    elseif strcmpi(smooth_method, 'Linear')
        % --- 线性比例分摊阶段 ---
        for j = 1:num_pts
            scale = j / num_pts;
            xs(j, :) = scale * xk_final';
        end
    else
        error('未知的平滑方法！请选择 ''RTS'' 或 ''Linear''');
    end
    
    % 统一提取桥接误差：时间轴起点的误差用于传给上一个 block
    bridge_error = xs(1, :)';
    
    % =====================================================================
    % 第二步：应用误差序列，修正状态并组装输出
    % =====================================================================
    nav_matrix = zeros(11, num_pts);
    smoothed_state_buffer = zeros(size(state_buffer));
    
    for j = 1:num_pts
        % 提取当前时刻的各向误差
        curr_xs = xs(j, :)';
        curr_pos_err = curr_xs(1:3);
        curr_vel_err = curr_xs(4:6);
        % curr_att_err = curr_xs(7:9); 
        
        % 1. 位置修正 (区分单位)
        if strcmpi(pos_unit, 'm')
            % 若误差是米制，通过地球半径矩阵转为纬经度弧度修正量
            [rm, rn] = getRmRn(state_buffer(j, 2), param);
            rm_plus_h = rm + state_buffer(j, 4);
            rn_plus_h = rn + state_buffer(j, 4);
            cos_lat = cos(state_buffer(j, 2));
            
            % 直接解析法求逆，比 inv(DR) 提速极多
            DR_inv = diag([1 / rm_plus_h, 1 / (rn_plus_h * cos_lat), -1]);
            curr_pos_rad = state_buffer(j, 2:4)' - DR_inv * curr_pos_err;
        else
            % 若误差已经是弧度/米组合，直接相减
            curr_pos_rad = state_buffer(j, 2:4)' - curr_pos_err;
        end
        
        % 2. 速度和姿态修正
        curr_vel = state_buffer(j, 5:7)' - curr_vel_err;
        
        % 根据你的原 RTS 代码，姿态暂未加入修正，保留原始补偿结构
        curr_att_rad = state_buffer(j, 8:10)' - zeros(3,1); 
        
        % 3. 准备输出
        % [rad -> deg] 转换
        pos_deg = [curr_pos_rad(1:2) * param.R2D; curr_pos_rad(3)];
        vel_ned = curr_vel;
        att_deg = curr_att_rad * param.R2D;
        
        % 填充平滑后的 state_buffer 结构
        smoothed_state_buffer(j, :) = [state_buffer(j, 1), curr_pos_rad', curr_vel', curr_att_rad'];
        
        % 填充给 fprintf 使用的 11 列输出矩阵
        nav_matrix(:, j) = [ ...
            range_idx; ...
            state_buffer(j, 1); ...
            pos_deg(1); pos_deg(2); pos_deg(3); ...
            vel_ned(1); vel_ned(2); vel_ned(3); ...
            att_deg(1); att_deg(2); att_deg(3) ...
        ];
    end
end

function kf = hard_constrain(navstate, pos, imudt, kf, MaxLimit)
% --- 优化后的分维度动态约束逻辑 ---

% 1. 获取当前时刻的动态最大允许偏差 (基于当前速度)
% 考虑到水下载体惯性，单次 IMU 周期内的位移修正不应无限制
v_n = abs(navstate.vel(1)); % 北向速度
v_e = abs(navstate.vel(2)); % 东向速度


% 阈值 = 基础值(8m) + 运动增量
limit_x = max(MaxLimit,v_n * imudt * 10); 
limit_y = max(MaxLimit,v_e * imudt * 10);

Re = 6378137;
% 2. 计算修正量在地理坐标系下的投影 (米)
% kf.x(1)对应纬度误差(rad)，kf.x(2)对应经度误差(rad)

dx_m = (pos(1)-(navstate.pos(1)- kf.x(1))) * Re;
dy_m = (pos(2)-(navstate.pos(2)- kf.x(2))) * Re * cos(pos(1));
drange = norm([dx_m,dy_m]);
% 3. 分维度判定与比例缩放
% 纬度(X)维度约束
if abs(dx_m) > limit_x
    % 计算超出比例：ratio > 1
    ratio_x = abs(dx_m) / limit_x;
    % 核心逻辑：将修正量压缩回 limit_x 的 0.7 倍左右，保证稳定性
    kf.x(1) = kf.x(1) / (ratio_x * 1.5); 
    % 只要产生了这种过于不准确的预判，降低协方差，代表此时误差并没有这么地大
    % kf.P(1,1) = kf.P(1,1) / 2; 
end

% 经度(Y)维度约束
if abs(dy_m) > limit_y
    ratio_y = abs(dy_m) / limit_y;
    kf.x(2) = kf.x(2) / (ratio_y * 1.5);
    % kf.P(2,2) = kf.P(2,2) / 2;
end

if drange > MaxLimit
    kf.x = 0.5 *kf.x;
end

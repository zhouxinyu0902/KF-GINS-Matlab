%% 反向推算+滤波,每个测距周期
function [NAV,navdt1s,indexrecord2,ki2]=backward_1s(runArgs)

imudata      = runArgs.imudata;
imuindex     = runArgs.imuindex;
ki2          = runArgs.ki2;
kf           = runArgs.kf;
navstate     = runArgs.navstate;
indexrecord2 = runArgs.indexrecord2;
rangedata    = runArgs.rangedata;
navdt1s      = runArgs.navdt1s;
NAV          = runArgs.NAV;
height       = runArgs.height;
rangeindex   = runArgs.rangeindex;
navstate0    = runArgs.navstate0;
meas         = runArgs.meas;

R2D = 180/pi;
thisimu2 = imudata(imuindex-1, :)';
lastimu2 = imudata(imuindex, :)';
navstate_2 = navstate;


kf2 = kf;
% kf2.P = kf.P/100;
% % kf2.P(7:9,7:9) = kf.P(7:9,7:9)*100;

ki2 = ki2 + 1;

nav = zeros(11, 1);
nav(2, 1) = navstate_2.time;
nav(3:5, 1) = [navstate_2.pos(1) * R2D; navstate_2.pos(2) * R2D; navstate_2.pos(3)];
nav(6:8, 1) = navstate_2.vel;
nav(9:11, 1) = navstate_2.att * R2D;
navdt1s(imuindex-1,:) = nav;

indexrecord2(ki2) = imuindex; % 记录旋转点
for ii = indexrecord2(ki2)-2:-1:indexrecord2(ki2-1)
    laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
    laststate_2.pos(3) = height(ii,2);
    kf2 = myInsPropagate_15state(laststate_2, thisimu2, 0.01, kf2);
    lastimu2 = thisimu2;
    thisimu2 = imudata(ii, :)';
    navstate_2 = laststate_2;

    nav = zeros(11, 1);
    nav(2, 1) = navstate_2.time;
    nav(3:5, 1) = [navstate_2.pos(1) * R2D; navstate_2.pos(2) * R2D; navstate_2.pos(3)];
    nav(6:8, 1) = navstate_2.vel;
    nav(9:11, 1) = navstate_2.att * R2D;
    navdt1s(ii,:) = nav;
    if ii == indexrecord2(ki2-1)
        if rangeindex == 2
            navstate0.time =  navstate0.time + 0.01;
            navstate_2 = navstate0;
        else
            % 反向滤波
            Rangedata = rangedata(rangeindex-2,:);
            depthdata = height(ii,1:2);
            % kf2 = myRangeUpdate(laststate_2, Rangedata, depthdata, kf2);
            if meas == "Range"
                kf2 = myRangeUpdate(laststate_2, Rangedata, depthdata, kf2);
            elseif meas == "Range+azi"
                kf2 = myRange_aziUpdate(laststate_2, Rangedata, depthdata, kf2);
            elseif meas == "Range+azi+pos"
                kf2 = myRange_azi_pos_Update(laststate_2, Rangedata, depthdata, kf2);
                % kf = myAziRangePosUpdate(navstate, Rangedata, depthdata, kf);
            end
            % kf2 = hard_constrain(laststate_2,lastimu2(1)-thisimu2(1), kf2, 100);
            [kf2, navstate_2] = myErrorFeedback_range(kf2, laststate_2);

        end
        nav = zeros(11, 1);
        nav(2, 1) = navstate_2.time;
        nav(3:5, 1) = [navstate_2.pos(1) * R2D; navstate_2.pos(2) * R2D; navstate_2.pos(3)];
        nav(6:8, 1) = navstate_2.vel;
        nav(9:11, 1) = navstate_2.att * R2D;
        navdt1s(ii,:) = nav;
    end

end

if ki2 == 2 % 首次，只对第一条轨迹进行滤波点旋转
    idstart = indexrecord2(ki2-1)+1;
    idend = indexrecord2(ki2)-1;
    index = idstart:idend;

    rotatepoint = [d2r(navdt1s(idstart-1,3:4)),0]';% 旋转点
    trajectory = flip([d2r(navdt1s(index,3:4)), zeros(length(index),1)]',2); % 待旋转轨迹
    [rotatedTrajectory, ~, ~] = rotateAndScaleTrajectory(trajectory, rotatepoint);
    rotatedTrajectory = flip(rotatedTrajectory, 2);
    rotatedTrajectory = [rotatepoint,rotatedTrajectory];

    rotatedTrajectory(1:2,:) = r2d(rotatedTrajectory(1:2,:));
    navdt1srotate = navdt1s(idstart-1:idend,:);
    navdt1srotate(:,3:4) = rotatedTrajectory(1:2,:)';
    NAV(idstart-1:idend,:) = navdt1srotate;

    % figure
    % plot(navdt1srotate(:,4),navdt1srotate(:,3),'r-')
    % hold on
    % plot(r2d(trajectory(2,:)),r2d(trajectory(1,:)),'g-')
    % plot(r2d(rotatepoint(2,:)),r2d(rotatepoint(1,:)),'*')
    % legend('旋转后','旋转前','旋转点')
else % 其余除了对上一条的轨迹进行尾部新的滤波点旋转，然后再对新的轨迹进行首部旋转
    idstart = indexrecord2(ki2-1)+1;
    idend = indexrecord2(ki2)-1;
    index = idstart:idend;

    last_idstart = indexrecord2(ki2-2);
    last_idend = indexrecord2(ki2-1)-1;
    last_index = last_idstart:last_idend;

    rotatepoint = [d2r(navdt1s(idstart-1,3:4)),0]';% 旋转点

    last_trj =[d2r(NAV(last_index,3:4)),zeros(length(last_index),1)]' ;
    [rotatedTrajectory, ~, ~] = rotateAndScaleTrajectory(last_trj, rotatepoint);

    rotatedTrajectory(1:2,:) = r2d(rotatedTrajectory(1:2,:));
    navdt1srotate = navdt1s(last_index,:);
    navdt1srotate(:,3:4) = rotatedTrajectory(1:2,:)';
    NAV(last_index,:) = navdt1srotate;
    % if ki2==1000
    %     keyboard;
    % end
    % if ki2==1598
    %     keyboard;
    % end
    % plot(navdt1srotate(:,4),navdt1srotate(:,3),'m')
    % hold on
    % plot(r2d(last_trj(2,:)),r2d(last_trj(1,:)),'b')
    % plot(r2d(rotatepoint(2,:)),r2d(rotatepoint(1,:)),'*')
    % % legend('旋转后','旋转前','旋转点')

    trajectory = flip([d2r(navdt1s(index,3:4)), zeros(length(index),1)]',2); % 待旋转轨迹
    [rotatedTrajectory, ~, ~] = rotateAndScaleTrajectory(trajectory, rotatepoint);
    rotatedTrajectory = flip(rotatedTrajectory, 2);
    rotatedTrajectory = [rotatepoint,rotatedTrajectory];

    rotatedTrajectory(1:2,:) = r2d(rotatedTrajectory(1:2,:));
    navdt1srotate = navdt1s(idstart-1:idend,:);
    navdt1srotate(:,3:4) = rotatedTrajectory(1:2,:)';
    NAV(idstart-1:idend,:) = navdt1srotate;

end
end


function [rotatedTrajectory, scaleFactor, rotationAngle_deg] = rotateAndScaleTrajectory(trajectory, newEndPoint)
% rotateAndScaleTrajectory: 将轨迹进行伸缩和旋转，使起点不变，终点与新终点重合。
%
%   输入:
%     trajectory   : 3×n矩阵，每行[经度(rad), 纬度(rad), 高度(m)]。
%     newEndPoint  : 3×1向量，[新终点经度(rad), 新终点纬度(rad), 高度(m)]。
%
%   输出:
%     rotatedTrajectory : n×3矩阵，旋转伸缩后的轨迹 [经度(rad), 纬度(rad), 高度(m)]。
%     scaleFactor       : 轨迹的伸缩因子 S。
%     rotationAngle_deg : 轨迹旋转的角度 (度)。
%
% 依赖的辅助函数定义:
% pos2dxyz(pos, pos0) : pos: n×3 [Lon, Lat, Alt] (rad/m), pos0: 3×1 [Lon, Lat, Alt] (rad/m) -> dxyz: n×3 [E, N, U] (m)
% dxyz2pos(dxyz, pos0) : dxyz: n×3 [E, N, U] (m), pos0: 3×1 [Lon, Lat, Alt] (rad/m) -> pos: n×3 [Lon, Lat, Alt] (rad/m)

% ----------------- 0. 输入格式检查与预处理 -----------------

% 将 3xn 轨迹转置为 n x 3 形式 [Lon, Lat, Alt] 以便进行基于行向量的计算
% (pos2dxyz/dxyz2pos需要 n x 3 输入, pos0需要 3 x 1 输入)
trajectory_n3 = trajectory'; % n x 3: [Lon, Lat, Alt]
n = size(trajectory_n3, 1);

if n < 2
    error('轨迹至少需要包含起点和终点两个点。');
end

% 提取起点和原终点的大地坐标
startPos_rad_m = trajectory_n3(1, :)';     % 3x1: [Lon, Lat, Alt]
originalEndPos_rad_m = trajectory_n3(end, :); % 1x3: [Lon, Lat, Alt]
newEndPos_rad_m = newEndPoint';              % 1x3: [Lon, Lat, Alt]

% ----------------- 1. 统一坐标系转换 (大地坐标 -> 局部XYZ坐标) -----------------

% 将所有点和新终点转换为相对于起点的局部XYZ坐标 (米)
% pos2dxyz(pos: n x 3, pos0: 3 x 1) -> dxyz: n x 3 [E, N, U] (m)
xyzpoints_n3 = pos2dxyz(trajectory_n3, startPos_rad_m); % n x 3: [E, N, U] (m)

% 只有新终点一个点，将其大地坐标转换为相对于起点的局部XYZ坐标
newEnd_dxyz_temp = pos2dxyz(newEndPos_rad_m, startPos_rad_m);
newEnd_dxyz = newEnd_dxyz_temp(1, :); % 1 x 3: [E, N, U] (m)

% 提取起点、原终点的局部坐标
startPoint_dxyz = xyzpoints_n3(1, :);        % 应为 [0, 0, 0]
originalEndPoint_dxyz = xyzpoints_n3(end, :);  % 1 x 3

% ----------------- 2. 计算伸缩因子 -----------------

% 向量：起点到原终点、起点到新终点
vecAB = originalEndPoint_dxyz - startPoint_dxyz; % 原向量
vecAC = newEnd_dxyz - startPoint_dxyz;           % 目标向量

% 向量长度（欧氏距离）
originalLength = norm(vecAB);
newLength = norm(vecAC);

% 伸缩因子 S
if originalLength < 1e-9
    if newLength > 1e-9
        error('起点和原终点重合 (距离为零)，无法伸缩到新终点。');
    else
        scaleFactor = 1; % 终点重合且新终点也重合，无需伸缩
    end
else
    scaleFactor = newLength / originalLength;
end

% ----------------- 3. 计算旋转矩阵 -----------------

% 归一化向量 (如果长度不为零)
if originalLength < 1e-9 || newLength < 1e-9
    % 至少有一个点与起点重合，不需要旋转
    rotationMatrix = eye(3);
    theta = 0;
else
    vecAB_norm = vecAB / originalLength;
    vecAC_norm = vecAC / newLength;

    % 计算旋转角度和轴
    cosTheta = dot(vecAB_norm, vecAC_norm);

    % 确保 cosTheta 在 [-1, 1] 范围内
    cosTheta = max(-1, min(1, cosTheta));

    theta = acos(cosTheta); % 旋转角度 (rad)
    rotationAngle_deg = rad2deg(theta);

    if abs(theta) < 1e-9 % 角度极小 (或平行同向)
        rotationMatrix = eye(3);
    elseif abs(theta - pi) < 1e-9 % 角度接近 180度 (反向)
        % 绕 Z (Up) 轴旋转 180 度，防止 cross 接近零
        rotationAxis = [0, 0, 1];
        rotationMatrix = diag([-1, -1, 1]); % 简化 180 度旋转矩阵
    else
        % 罗德里格斯公式
        rotationAxis = cross(vecAB_norm, vecAC_norm);
        rotationAxis = rotationAxis / norm(rotationAxis); % 归一化
        sinTheta = sin(theta);

        K = [0, -rotationAxis(3), rotationAxis(2);
            rotationAxis(3), 0, -rotationAxis(1);
            -rotationAxis(2), rotationAxis(1), 0];

        rotationMatrix = eye(3) + sinTheta * K + (1 - cosTheta) * (K * K);
    end
end

% ----------------- 4. 应用变换 (旋转 + 伸缩) -----------------
rotatedPoints_dxyz_n3 = zeros(n, 3);
for i = 1:n
    % 1. 平移至原点 (StartPoint_dxyz 应为 [0, 0, 0])
    p_orig_shifted = xyzpoints_n3(i, :)'; % 3x1 向量

    % 2. 旋转
    p_rotated = rotationMatrix * p_orig_shifted;

    % 3. 伸缩
    p_scaled = p_rotated * scaleFactor;

    % 4. 平移回原点 (起点): P_new = p_scaled + startPoint_dxyz
    % 由于 startPoint_dxyz = [0, 0, 0]，所以 P_new = p_scaled
    rotatedPoints_dxyz_n3(i, :) = p_scaled'; % 存回 n x 3 矩阵
end

% ----------------- 5. 逆转换：局部XYZ(m) -> 经纬度(rad) -----------------

% dxyz2pos(dxyz: n x 3, pos0: 3 x 1) -> pos: n x 3 [Lon, Lat, Alt] (rad/m)
rotatedTrajectory = dxyz2pos(rotatedPoints_dxyz_n3, startPos_rad_m); % n x 3: [Lon, Lat, Alt]
rotatedTrajectory=rotatedTrajectory';
% ----------------- 6. 结果结构体 (eval_results) -----------------
eval_results.scaleFactor = scaleFactor;
eval_results.rotationAngle_deg = rotationAngle_deg;

% ----------------- 7. 结果验证和显示 (可选) -----------------

if nargout == 0 % 如果函数没有输出，打印结果
    fprintf('轨迹伸缩和旋转完成。\n');
    fprintf('伸缩因子 (S): %.4f\n', scaleFactor);
    fprintf('旋转角度 (Theta): %.2f 度\n', rotationAngle_deg);

    % 验证终点是否重合（在局部坐标系中）
    finalEnd_dxyz = rotatedPoints_dxyz_n3(end, :);
    errorVec = finalEnd_dxyz - newEnd_dxyz;
    endPointError = norm(errorVec);
    fprintf('终点重合误差 (XYZ): %.6f m\n', endPointError);

    % 验证终点是否重合（在大地坐标系中）
    finalEnd_pos = rotatedTrajectory(end, :);
    posError_lon = finalEnd_pos(1) - newEndPos_rad_m(1);
    posError_lat = finalEnd_pos(2) - newEndPos_rad_m(2);
    posError_alt = finalEnd_pos(3) - newEndPos_rad_m(3);
    fprintf('终点经度误差: %.6f rad\n', posError_lon);
    fprintf('终点纬度误差: %.6f rad\n', posError_lat);
end

end
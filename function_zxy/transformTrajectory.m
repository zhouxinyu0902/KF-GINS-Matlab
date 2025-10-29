function transformedTrajectory = transformTrajectory(trajectory)
    % trajectory: n×2矩阵，每行[经度, 纬度]（单位：rad）
    % 输出：变换后的轨迹，终点与起点重合
    
    % 转置为列向量
    trajectory = trajectory';
    
    % 将轨迹点转换为ECEF坐标（以起点为参考）
    xyzpoints = pos2dxyz(trajectory, trajectory(1,:)');
    
    % 提取起点和终点
    startPoint = xyzpoints(1, :);
    endPoint = xyzpoints(end, :);
    
    % 计算起点到终点的向量
    vecSE = endPoint - startPoint;
    distanceSE = norm(vecSE);
    
    % 计算旋转轴（起点-终点连线方向）
    if distanceSE > eps
        rotationAxis = vecSE / distanceSE;
    else
        % 如果起点终点重合，不需要旋转
        rotationAxis = [0, 0, 1]; % 默认Z轴
    end
    
    % 计算旋转角度（180度，使终点旋转到起点位置）
    rotationAngle = pi; % 180度
    
    % 构建旋转矩阵（罗德里格斯公式）
    K = [0, -rotationAxis(3), rotationAxis(2);
         rotationAxis(3), 0, -rotationAxis(1);
         -rotationAxis(2), rotationAxis(1), 0];
    rotationMatrix = eye(3) + sin(rotationAngle) * K + (1 - cos(rotationAngle)) * (K * K);
    
    % 变换轨迹点（绕起点进行伸缩和旋转）
    transformedPoints = zeros(size(xyzpoints));
    for i = 1:size(xyzpoints, 1)
        % 计算当前点到起点的向量
        vecSP = xyzpoints(i, :) - startPoint;
        
        % 计算当前点在SE方向上的投影比例
        if distanceSE > eps
            t = dot(vecSP, vecSE) / (distanceSE * distanceSE);
        else
            t = 0;
        end
        
        % 计算缩放因子（起点处为1，终点处为0）
        s = 1 - t;
        
        % 应用旋转和缩放
        rotatedVec = (rotationMatrix * vecSP')';
        transformedVec = s * rotatedVec;
        
        % 得到变换后的点
        transformedPoints(i, :) = startPoint + transformedVec;
    end
    
    % 将变换后的ECEF坐标转换回经纬度(rad)
    transformedTrajectory = dxyz2pos(transformedPoints, trajectory(1,:)');
    transformedTrajectory = transformedTrajectory';
end
function rotatedTrajectory = rotateTrajectory(trajectory, newEndPoint)
    % trajectory: n×2矩阵，每行[经度, 纬度]（单位：rad）
    % newEndPoint: 1×2向量，[新终点经度, 新终点纬度]（单位：rad）
    trajectory=trajectory';
    newEndPoint=newEndPoint';
    n = length(trajectory);
    % 将轨迹点转换为ECEF坐标
    xyzpoints = pos2dxyz(trajectory, trajectory(1,:)');
    
    % 提取起点、原终点和新终点
    startPoint = xyzpoints(1, :);
    originalEndPoint = xyzpoints(end, :);
    newEndPoint_ecef = pos2dxyz(newEndPoint, trajectory(1,:)');
    
    % 计算向量：起点到原终点、起点到新终点
    vecAB = originalEndPoint - startPoint;
    vecAC = newEndPoint_ecef - startPoint;
    
    % 归一化向量
    vecAB = vecAB / norm(vecAB);
    vecAC = vecAC / norm(vecAC);
    
    % 计算旋转轴（叉积）和旋转角度（点积）
    rotationAxis = cross(vecAB, vecAC);
    rotationAxis = rotationAxis / norm(rotationAxis); % 归一化
    cosTheta = dot(vecAB, vecAC);
    sinTheta = norm(cross(vecAB, vecAC));
    theta = atan2(sinTheta, cosTheta); % 旋转角度
    
    % 构建旋转矩阵（罗德里格斯公式）
    K = [0, -rotationAxis(3), rotationAxis(2);
         rotationAxis(3), 0, -rotationAxis(1);
         -rotationAxis(2), rotationAxis(1), 0];
    rotationMatrix = eye(3) + sinTheta * K + (1 - cosTheta) * (K * K);
    
    % 旋转轨迹点（绕起点旋转）
    rotatedPoints = zeros(size(xyzpoints));
    for i = 1:size(xyzpoints, 1)
        % 平移至原点，旋转，再平移回原位置
        rotatedPoints(i, :) = startPoint + (rotationMatrix * (xyzpoints(i, :) - startPoint)')';
    end
    
    % 将旋转后的ECEF坐标转换回经纬度(rad)
    rotatedTrajectory = dxyz2pos(rotatedPoints, trajectory(1,:)');
    rotatedTrajectory = rotatedTrajectory';
end

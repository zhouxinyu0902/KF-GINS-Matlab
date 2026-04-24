function [pos] = LBL_Positioning(distances, stations)
% LBL_POSITIONING 长基线水平定位函数
% 输入:
%   distances: 1x3 向量，待定位点到三个基站的水平距离 [d1, d2, d3]
%   stations:  3x2 矩阵，三个基站的水平坐标 [x1, y1; x2, y2; x3, y3]
% 输出:
%   pos: 1x2 向量，计算出的待定位点坐标 [x, y]

    % 提取坐标和距离
    x1 = stations(1,1); y1 = stations(1,2); d1 = distances(1);
    x2 = stations(2,1); y2 = stations(2,2); d2 = distances(2);
    x3 = stations(3,1); y3 = stations(3,2); d3 = distances(3);

    % 构建线性方程组 A*X = B
    % 2(x1 - x2)x + 2(y1 - y2)y = d2^2 - d1^2 - x2^2 + x1^2 - y2^2 + y1^2
    % 2(x1 - x3)x + 2(y1 - y3)y = d3^2 - d1^2 - x3^2 + x1^2 - y3^2 + y1^2
    
    A = [ 2*(x1 - x2), 2*(y1 - y2);
          2*(x1 - x3), 2*(y1 - y3) ];
      
    B = [ d2^2 - d1^2 - x2^2 + x1^2 - y2^2 + y1^2;
          d3^2 - d1^2 - x3^2 + x1^2 - y3^2 + y1^2 ];

    % 使用最小二乘法求解 (A\B 在超静定或正常情况下均适用)
    pos_column = A \ B;
    
    % 转置为行向量输出
    pos = pos_column';
end
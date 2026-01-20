function plot_xk(xkpath, navpath, truthpath)
    % PLOT_XK: 绘制导航误差与卡尔曼滤波估计状态对比图
    
    %% 1. 加载数据与初始化
    param = Param();
    nav   = importdata(navpath);
    ref   = importdata(truthpath);
    xk    = importdata(xkpath);
    
    % 使用 intersect 快速对齐时间戳 (假设第二列是时间戳)
    [commonTimes, iNav, iRef] = intersect(nav(:,2), ref(:,2));
    nav = nav(iNav, :);
    ref = ref(iRef, :);
    xk  = xk(iNav, :);  % 假设 xk 与 nav 时间同步
    
    %% 2. 误差计算与坐标转换 (LLA to NED Error)
    % 计算位置、速度、姿态误差
    err = zeros(size(nav, 1), 10);
    err(:, 1) = commonTimes;
    % 原始差值：nav(lat, lon, h, VN, VE, VD, roll, pitch, yaw)
    err(:, 2:10) = nav(:, 3:11) - ref(:, 3:11);
    
    % 航向角过零处理 (Heading wrap-around)
    err(:, 10) = atan2(sin(err(:, 10) * param.D2R), cos(err(:, 10) * param.D2R)) * param.R2D;
    
    % 获取子午圈/卯酉圈半径 (以起始点参考)
    [rm, rn] = getRmRn(ref(1,3) * param.D2R, param);
    h0 = ref(1,5);
    cosLat = cos(ref(1,3) * param.D2R);

    % 将纬度/经度弧度误差转换为米 (xk通常输出的是米或弧度，这里统一转为米)
    % 转换 err (真值误差)
    err(:, 2) = err(:, 2) * param.D2R * (rm + h0);       % dLat -> dNorth (m)
    err(:, 3) = err(:, 3) * param.D2R * (rn + h0) * cosLat; % dLon -> dEast (m)
    
    % 转换 xk (滤波器估计误差)
    % 注意：需确认你滤波器输出的 xk 前两维是弧度还是米。如果是弧度则需如下转换：
    xk(:, 2) = xk(:, 2) * (rm + h0); 
    xk(:, 3) = xk(:, 3) * (rn + h0) * cosLat;
    
    % 角度单位转换 (Rad -> Deg)
    err(:, 8:10) = err(:, 8:10); % err 已经是度了（假设nav/ref输入是度）
    xk(:, 8:10)  = xk(:, 8:10) * param.R2D;

    %% 3. 绘制 9 状态误差对比图 (位置、速度、姿态)
    titles = {'North Pos Error (m)', 'East Pos Error (m)', 'Down Pos Error (m)', ...
              'North Vel Error (m/s)', 'East Vel Error (m/s)', 'Down Vel Error (m/s)', ...
              'Roll Error (deg)', 'Pitch Error (deg)', 'Heading Error (deg)'};
    
    figure('Name', 'Navigation Error vs Filter Estimation', 'Color', 'w');
    myfigurestartup(10, 10, 'prese');
    
    for i = 1:9
        subplot(3, 3, i);
        plot(err(:,1), err(:, i+1), 'r', 'LineWidth', 1); hold on;
        plot(xk(:,1),  xk(:, i+1),  'b--', 'LineWidth', 1);
        grid on;
        title(titles{i});
        if i == 1, legend('True Error', 'Estimated', 'Location', 'best'); end
    end

    %% 4. 绘制传感器零偏 (IMU Bias)
    % 陀螺零偏转换: rad/s -> deg/h
    eb = xk(:, 11:13) * param.R2D * 3600;
    % 加计零偏转换: m/s^2 -> mGal (1e-5 m/s^2)
    db = xk(:, 14:16) / 1e-5;

    figure('Name', 'IMU Bias Estimation', 'Color', 'w');
    
    % 陀螺零偏 Eb
    labels_eb = {'eb_x (deg/h)', 'eb_y (deg/h)', 'eb_z (deg/h)'};
    for i = 1:3
        subplot(2, 3, i);
        plot(xk(:,1), eb(:, i), 'Color', [0 0.5 0]); grid on;
        ylabel(labels_eb{i}); title(['Gyro Bias ', num2str(i)]);
    end
    
    % 加计零偏 Db
    labels_db = {'db_x (mGal)', 'db_y (mGal)', 'db_z (mGal)'};
    for i = 1:3
        subplot(2, 3, i+3);
        plot(xk(:,1), db(:, i), 'Color', [0.6 0.2 0]); grid on;
        ylabel(labels_db{i}); title(['Acc Bias ', num2str(i)]);
    end
end
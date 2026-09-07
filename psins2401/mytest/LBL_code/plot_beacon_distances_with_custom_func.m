function plot_beacon_distances_with_custom_func(BCNddm,BCNrrm,trjddm)
% beacon_coords: Nx3矩阵，每行为[纬度(度), 经度(度), 深度(m)]
% 使用自定义的caldot2dot函数计算距离

% 检查输入
if size(BCNddm,2) ~= 3
    error('输入矩阵应为Nx3格式：[纬度,经度,深度]');
end

figure;
plot(BCNddm(:,2), BCNddm(:,1), '*', 'MarkerSize', 10);
hold on;
plot(trjddm(:,2), trjddm(:,1),  'b-', 'LineWidth', 1.5);
% 计算并显示信标间的距离差
num_bcn = size(BCNddm, 1);
for i = 1:num_bcn
    for j = i+1:num_bcn
        % 计算水平距离差（经纬度转换为米）
        ll1 = BCNrrm(i,1:3);
        ll2 = BCNrrm(j,1:3); 
        [~,horizontal_dist] = caldot2dot(ll1,ll2); % 转换为米
        
        % 计算深度差
        depth_diff = abs(BCNrrm(i,3) - BCNrrm(j,3));
        
        % 计算中点坐标用于文本标注
        mid_x = (BCNddm(i,2) + BCNddm(j,2))/2;
        mid_y = (BCNddm(i,1) + BCNddm(j,1))/2;
        mid_z = (BCNddm(i,3) + BCNddm(j,3))/2;
        
        % 绘制连接线
        line([BCNddm(i,2), BCNddm(j,2)], [BCNddm(i,1), BCNddm(j,1)], [BCNddm(i,3), BCNddm(j,3)], ...
             'Color', [0.5 0.5 0.5], 'LineStyle', '--');
        
        % 标注距离信息
        text(mid_x, mid_y, mid_z, ...
             sprintf('水平差: %.1fm\n深度差: %.1fm', horizontal_dist, depth_diff), ...
             'FontSize', 8, 'Color', 'k', 'BackgroundColor', 'w');
    end
end
% 设置图形属性
xlabel('经度'); ylabel('纬度'); zlabel('深度(m)');
title('信标分布与航行器轨迹（标注信标间距）');
legend('Location', 'best');
grid on;
legend('信标','轨迹')
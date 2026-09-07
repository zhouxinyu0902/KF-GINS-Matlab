function plot_result_and_error(BCNxzy,true_auv_xyz,auv_pos,horizontal_error,residual)
% 解算效果图绘制

% figure('Position', [100,100,800,600]);
% % 1. 绘制信标位置
% scatter(BCNxzy(:,1), BCNxzy(:,2), 100, 'k^', 'filled', 'DisplayName', '信标');
% hold on;
% 
% % 2. 绘制真实轨迹（假设存在）
% if exist('true_auv_xyz','var')
%     plot(true_auv_xyz(:,1), true_auv_xyz(:,2), 'b.',...
%           'LineWidth',1.5, 'DisplayName','真实轨迹');
% end
% 
% % 3. 绘制解算轨迹
% plot(auv_pos(:,1), auv_pos(:,2), 'r.',...
%       'LineWidth',1.5, 'MarkerSize',6, 'DisplayName','解算轨迹');
% 
% % % 4. 标注典型点
% % for k = [1, round(num_samples/2), num_samples]
% %     text(auv_pos(k,1), auv_pos(k,2),...
% %         sprintf('t=%d\nres=%.2fm',k,residual(k)),...
% %         'VerticalAlignment','bottom', 'FontSize',8);
% % end
% 
% % 图形设置
% xlabel('UTM东向坐标(m)'); ylabel('UTM北向坐标(m)'); zlabel('深度(m)');
% title('水下航行器定位解算效果（深度已知）');
% legend('Location','best'); grid on; axis equal;

% 附加残差分析图
myfigurestartup(10,5,'prese')
subplot 121
plot(residual, 'LineWidth',1.5);
xlabel('时间步'); ylabel('残差范数(m)');
title('解算残差变化曲线');
% 水平误差
subplot 122
plot(horizontal_error(:,1), horizontal_error(:,2), 'r-');
xlabel('时间(s)'); ylabel('误差(m)');
title('水平定位误差');
grid on;

% 显示统计结果
fprintf('定位性能统计:\n');
fprintf('平均水平误差: %.2f ± %.2f m\n', mean(horizontal_error(:,2)), std(horizontal_error(:,2)));
fprintf('最大水平误差: %.2f m\n\n', max(horizontal_error(:,2)));
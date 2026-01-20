clear all
glvs

nav=importdata("旋转收缩方案\input\truth.nav");
posauv_ddm=nav(:,3:5);
posauv_rrm=nav(:,3:5);
posauv_rrm(:,1:2)=d2r(posauv_rrm(:,1:2));

bcn1=importdata("旋转收缩方案\input\range1.txt");
bcn2=importdata("旋转收缩方案\input\range2.txt");
bcn3=importdata("旋转收缩方案\input\range3.txt");

beacon1_rrm=bcn1(1,4:6);
beacon2_rrm=bcn2(1,4:6);
beacon3_rrm=bcn2(1,4:6);

beacon1_mmm=pos2dxyz(beacon1_rrm,posauv_rrm(1,:)');
beacon2_mmm=pos2dxyz(beacon2_rrm,posauv_rrm(1,:)');
beacon3_mmm=pos2dxyz(beacon3_rrm,posauv_rrm(1,:)');

auv_mmm=pos2dxyz(posauv_rrm,posauv_rrm(1,:)');
auv_mmm=auv_mmm(100*60*1:100*60*1:end,:);
beacon_mmm=beacon3_mmm;
beacon=[beacon1_mmm;beacon2_mmm;beacon3_mmm];
for i=1:3
    k(i)=calculate_log_det_FIM(beacon(i,1:2), auv_mmm(:,1:2), 5);
end
% 计算可观测度
obs_results = calculate_instantaneous_observability(beacon_mmm(1:2), auv_mmm(:,1:2), 5);


% 绘制可观测度随时间变化
% figure('Position', [100, 100, 1200, 700]);
% 
% subplot(3,1,1);
% plot(obs_results.mu_D, 'b-', 'LineWidth', 2);
% ylabel('D-最优性指标 \mu_D');
% title('瞬时可观测度时间序列');
% grid on;
% 
% subplot(3,1,2);
% plot(obs_results.mu_C, 'r-', 'LineWidth', 2);
% ylabel('条件数指标 \mu_C');
% set(gca, 'YScale', 'log'); % 对数坐标更直观
% grid on;
% 
% subplot(3,1,3);
% plot(obs_results.range, 'g-', 'LineWidth', 2);
% xlabel('轨迹点索引');
% ylabel('距离 (m)');
% grid on;
% %%
% % 绘制轨迹与可观测度热图
% figure('Position', [100, 100, 1000, 400]);
% 
% subplot(1,2,1);
% scatter(auv_mmm(:,1), auv_mmm(:,2), 50, obs_results.mu_D, 'filled');
% hold on;
% plot(beacon_mmm(1), beacon_mmm(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% colorbar;
% xlabel('X坐标 (m)'); ylabel('Y坐标 (m)');
% title('轨迹可观测度热图 (D-最优性指标)');
% axis equal; grid on;
% 
% subplot(1,2,2);
% % 绘制方位角-距离-可观测度关系
% polarscatter(obs_results.bearing, obs_results.range, 50, obs_results.mu_D, 'filled');
% colorbar;
% title('极坐标下的可观测度分布');
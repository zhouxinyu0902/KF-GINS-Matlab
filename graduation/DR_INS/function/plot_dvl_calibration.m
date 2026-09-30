function fig = plot_dvl_calibration(path, true_scale, true_yaw_deg)
if nargin < 2, true_scale = nan; end
if nargin < 3, true_yaw_deg = nan; end
data = importdata(path);
t = data(:,1)-data(1,1);
DVLscale = data(:,2);
DVLyaw = data(:,3);
DVLscalestd = data(:,4);
DVLyawstd = data(:,5);
fig = myfigurestartup(7,2.5,'paper');
subplot(1,2,1); hold on;
plot(t,DVLscale,'LineWidth',1.2);
plot(t,DVLscale+DVLscalestd,'--','LineWidth',0.8);
plot(t,DVLscale-DVLscalestd,'--','LineWidth',0.8);
xlim([min(t),max(t)])
yline(0,'k:','LineWidth',0.8,'HandleVisibility','off');
if isfinite(true_scale)
    yline(true_scale, 'r-.', 'True value', 'LineWidth', 1.0);
end
grid on; box on;
xlabel('Time (s)'); ylabel('DVL scale factor');
title('DVL Scale Factor Calibration');
legend('Estimate','+1\sigma','-1\sigma','Location','best');
subplot(1,2,2); hold on;
plot(t,DVLyaw,'LineWidth',1.2);
plot(t,DVLyaw+DVLyawstd,'--','LineWidth',0.8);
plot(t,DVLyaw-DVLyawstd,'--','LineWidth',0.8);
xlim([min(t),max(t)])
yline(0,'k:','LineWidth',0.8,'HandleVisibility','off');
if isfinite(true_yaw_deg)
    yline(true_yaw_deg, 'r-.', 'True value', 'LineWidth', 1.0);
end
grid on; box on;
xlabel('Time (s)'); ylabel('DVL yaw misalignment');
title('DVL Yaw Misalignment Calibration');
legend('Estimate','+1\sigma','-1\sigma','Location','best');
end

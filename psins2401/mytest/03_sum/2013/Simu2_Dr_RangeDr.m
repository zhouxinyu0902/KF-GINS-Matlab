clear
load('data_1\data_dr.mat')
glvs
ts=trj.ts;

depther=-avp_ref(:,9)+normrnd(0,depthstd,size(avp_ref(:,9)));
rng(1)
% %% 仿真条件绘图
% myfigurestartup(5,5,'paper')
% insplot(trj.avp(:,[1:6,end]),'av')
% print(gcf, 'New Folder\仿真-姿态和速度.png', '-dpng','-r600');
% 
% myfigurestartup(7,3,'paper')
% subplot 121,insplot(trj.avp(:,[7:9,end]),'p')
% subplot 122,insplot(trj.avp(:,[7:8,end]),'l')
% print(gcf, 'New Folder\仿真-位移+轨迹.png', '-dpng','-r600');

%% 信标位置和距离
rngk=6;rngc=0;
[RNG,BCN]=beacon_gen(avp_ref(1,7:9)',rngk,rngc,avp_ref,8,0);
%% 信标+轨迹绘图
myfigurestartup(3,3,'paper')
dxyz = pos2dxyz([avp_ref(:,7:8),avp_ref(:,9)]);
dxyzship = pos2dxyz([BCN{9}(:,1),BCN{9}(:,2),BCN{9}(:,2)*0],avp_ref(1,7:9)');
dxyzship1 = pos2dxyz([BCN{10}(:,1),BCN{10}(:,2),BCN{10}(:,2)*0],avp_ref(1,7:9)');
plot(0, 0, 'rp');
hold on, 
plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
plot(dxyzship(1,1), dxyzship(1,2),'*');
plot(dxyzship(:,1), dxyzship(:,2));
% plot(dxyzship1(1,1), dxyzship1(1,2),'*');
% plot(dxyzship1(:,1), dxyzship1(:,2));
for i=1:4
    BCN1=pos2dxyz(BCN{i},avp_ref(1,7:9)');
    hold on
    plot(BCN1(1),BCN1(2),'*')
end
xx=xticks;
xticks(xx(1):500:xx(end))
yy=yticks;
yticks(yy(1):500:yy(end))
axis equal
legend('start','trajectory','start','moving beacon','beacon1','beacon2','beacon3','beacon4','Location','southwest')
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\仿真-信标+轨迹.png', '-dpng','-r600');
%% 画一个3D图
% figure
% plot3(dxyz(:,1),dxyz(:,2),dxyz(:,3))
% hold on, plot3(dxyzship(:,1), dxyzship(:,2),dxyzship(:,3));
% for i=1:4
%     BCN1=pos2dxyz(BCN{i},avp_ref(1,7:9)');
%     hold on
%     plot3(BCN1(1),BCN1(2),BCN1(3),'*')
% end
%% 初始值的设置也会影响距离辅助的效果
% x0=[0.05;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
% dx0=[0.01;d2r(1);1/glv.Re;1/glv.Re];

x0=[0;0;0;0];% 初始值
dx0=[0.05;d2r(0.5);1/glv.Re;1/glv.Re];% 初始值不确定性
%% 距离辅助（单个信标）
ii=9;
range=RNG{ii};
beacon=BCN{ii};
dr = mydr('init',avp_ref(1,7:9)',[1;1;0.2],ts);
kf = myekf('init',0.5, x0, dx0, [0,web,0,0], rngk);
[avp_range,avp_dr1,kk_1,xkpk]=prealloc(length(avp_ref),10,10,5,9);
ki=1;
for i=1:length(compass)
    t = compass(i,end);
    dr = mydr('update',dr,-depther(i),compass(i,3),vxy(i,1:2));
    avp_dr1(i,:)=[dr.avp',t];
    % 以上为航位推算部分
    kf = myekf('fk',kf, dr);
    kf = myekf('algo',kf,'T');
    if mod(t,8)==0
        % 使用水平距离
        if size(beacon,1)==1
            dr.beacon=beacon;
            r=sqrt(range(i)^2-(avp_ref(i,9)-dr.beacon(3))^2);% 测量值
        else
            dr.beacon=beacon(ki,:);
            r=range(ki);
        end
        kf.r_dr=sqrt(RCompu(dr.pos',dr.beacon)^2-(-depther(i)-dr.beacon(3))^2); % 计算值
        kf.yk=kf.r_dr-r;
        kf = myekf('hk',kf,dr,'range');
        kf = myekf('algo',kf,'M');
        
        % dr.pos(1:2) = dr.pos(1:2)-kf.xk(3:4);
        % kf.xk(3:4)=[0;0];
        % dr.kod = 1 + kf.xk(1);
        % kf.xk(1) = 0;
        
        % 计算可观测矩阵
        MK(:,:,ki)=kf.MK;
        LK(:,:,ki)=kf.Lk;
        Mk_instant(:,:,ki)=kf.Mk;
        y(ki,1:2)=[kf.yk,t];
        alpha(ki,1:2)=[kf.alpha,t];
        kk_1(ki,1:4)=kf.xk;
        kk_1(ki,5)=t;
        avp_range(ki,:) = [dr.avp', t];
        xkpk(ki,:)=[kf.xk',diag(kf.Pxk)',t];
        ki=ki+1;
    end
end
avp_range(ki:end,:) = [];
kk_1(ki:end,:) = [];
xkpk(ki:end,:) = [];
avp_range(:,7)=avp_range(:,7)-kk_1(:,3);
avp_range(:,8)=avp_range(:,8)-kk_1(:,4);
%% 单个信标绘图 径向误差+估计状态
myfigurestartup(5,3,'paper')
subplot 121,
trjsee(avp_ref,'2d',avp_dr,avp_range),legend('true trajectory','DR','DR/Range')
% axis equal
subplot 122, % 误差绘图
RadialError=RCompu(avp_ref(:,7:9),avp_dr(:,7:9));
RadialError_range=RCompu(avp_ref(16:16:end,7:9),avp_range(:,7:9));
plot(avp_ref(:,end),RadialError)
hold on
plot(avp_ref(16:16:end,end),RadialError_range)
legend('DR','DR/Range')
xlim([avp_ref(1,end) avp_ref(end,end)])
xygo('t/s','Error/m')
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\仿真-轨迹+径向误差.png', '-dpng','-r600');
% dr_err=avpcmp(avp_dr,avp_ref);
% myfigurestartup(5,5,'paper'),xk_plot(0.05,0.01,dr_err,'range-aided',{kk_1},0)
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\仿真-航位推算+距离误差估计.png', '-dpng','-r600');
myfigurestartup(3,3,'paper'),
plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(:,7:9)),'m')
hold on
plot(avp_range(:,end),RCompu(avp_ref(16:16:end,7:9),avp_range(:,7:9)));
% plot(y(:,2),y(:,1))
% plot(alpha(:,2),alpha(:,1))

%%
% 假设 LK 是存储了 I_j 或 M_j 的 4x4xn 矩阵
lambda_sub = zeros(2, length(LK)); % 用于存储排序后的特征值
V_small_sub = zeros(2, length(LK)); % 用于存储最小特征值对应的特征向量
V_big_sub = zeros(2, length(LK)); % 用于存储最小特征值对应的特征向量
for i = 1:length(LK)
    % 1. 提取子矩阵 I_j^{L\lambda} (2x2)
    I_sub = LK(3:4, 3:4, i);
    
    % 2. 计算特征值和特征向量
    % V_sub 是特征向量矩阵，D_sub 是对角特征值矩阵
    [V_sub, D_sub] = eig(I_sub);
    
    % 3. 提取特征值并排序
    lambda = diag(D_sub);
    [lambda_sorted, index] = sort(lambda, 'ascend'); % 升序排序
    
    % 4. 存储排序后的特征值
    lambda_sub(:, i) = lambda_sorted; 
    
    % 5. 提取最小特征值对应的特征向量 (V_small)
    % 最小特征值 (lambda_sorted(1)) 对应的索引是 index(1)
    index_of_min_lambda = index(1); 
    index_of_max_lambda = index(2); 
    % V_sub 的列是特征向量。我们使用 index(1) 来选择对应的列
    V_small = V_sub(:, index_of_min_lambda);
    V_big = V_sub(:, index_of_max_lambda);
    % 6. 存储最小特征向量 (弱观测方向)
    V_small_sub(:, i) = V_small;
    V_big_sub(:, i) = V_big;
end

% 结果解释：
% lambda_sub(1, i) 是最小特征值 (弱观测度方向)
% lambda_sub(2, i) 是最大特征值 (高观测度方向)
% figure
% subplot 121
% plot(abs(V_small_sub(1,:)),'.','DisplayName','lat')
% hold on
% plot(abs(V_small_sub(2,:)),'.','DisplayName','lon')
% legend();
% subplot 122
% plot(abs(V_big_sub(1,:)),'.','DisplayName','lat')
% hold on
% plot(abs(V_big_sub(2,:)),'.','DisplayName','lon')
% legend();
%%
err_range = avpcmp(avp_range,avp_ref);
err = avpcmp(avp_dr,avp_ref);
if length(beacon)>3
    beacon=beacon(1:length(avp_range),:);
end
% 可观测度和经度纬度差的对比关系
myfigurestartup(12,7,'prese')
subplot(1,2,1)
plot(avp_ref(16:16:end,end),abs(avp_ref(16:16:end,7)-beacon(:,1))*glv.Re,'b')
hold on
plot(avp_ref(16:16:end,end),abs(cos(18)*(avp_ref(16:16:end,8)-beacon(:,2)))*glv.Re,'r')
ylabel('轨迹与信标纬度/经度差/m')
yyaxis right
plot(8:8:8700,squeeze(LK(3,3,:)),'b--')
plot(8:8:8700,squeeze(LK(4,4,:)),'r--')
ylabel('可观测矩阵对角线元素')
legend('dlat','dlon','lat_{ob}','lon_{ob}')


err_pos=err(16:16:end,7:8);
est_err=abs(kk_1(:,3:4)-err_pos(:,:));
subplot(1,2,2)
plot(avp_ref(16:16:end,end),abs(avp_ref(16:16:end,7)-beacon(:,1))*glv.Re,'b')
hold on
plot(avp_ref(16:16:end,end),abs(cos(18)*(avp_ref(16:16:end,8)-beacon(:,2)))*glv.Re,'r')
ylabel('轨迹与信标纬度/经度差/m')
yyaxis right
plot(8:8:8700,est_err(:,1)*glv.Re,'b--')
plot(8:8:8700,cos(18)*est_err(:,2)*glv.Re,'r--')
ylabel('纬度/经度估计误差/m')
legend('dlat','dlon','lat_{err}','lon_{err}')

%%
figure
subplot 121
plot(8:8:8700,est_err(:,1)*glv.Re,'DisplayName','estimated-err_{lat}')
ylabel('纬度估计误差/m')
yyaxis right
% plot(8:8:8700,1./squeeze(LK(3,3,:)),'g--')
hold on
plot(8:8:8700,abs(V_small_sub(1,:)),'--','DisplayName','lat_{small}')
plot(8:8:8700,abs(V_big_sub(1,:)),'-.','DisplayName','lat_{big}')
ylabel('可观测度分析')
legend();
subplot 122
plot(8:8:8700,cos(18)*est_err(:,2)*glv.Re,'DisplayName','estimated-err_{lon}')
ylabel('经度估计误差/m')
yyaxis right
% plot(8:8:8700,1./squeeze(LK(4,4,:)),'r--')
hold on
plot(8:8:8700,abs(V_small_sub(2,:)),'--','DisplayName','lon_{small}')
plot(8:8:8700,abs(V_big_sub(2,:)),'-.','DisplayName','lat_{big}')
legend();
ylabel('可观测度分析')
xlabel('时间/s')
%%

figure
subplot 121
plot(err(:,end),err(:,7),kk_1(:,end),kk_1(:,3))
subplot 122
plot(err(:,end),err(:,8),kk_1(:,end),kk_1(:,4))



figure
subplot 121
plot(err_range(:,end),err_range(:,7),err(:,end),err(:,7))
subplot 122
plot(err_range(:,end),err_range(:,8),err(:,end),err(:,8))


%%

% myfigurestartup(5,5,'paper')
% xk_plot(0.05,d2r(0.5),dr_err,{'propa'},{xkpk(:,[1:4,end])},0)
%%


% figure
% plot(squeeze(MK(3,3,:)),'DisplayName','lat')
% hold on
% plot(squeeze(MK(1,1,:)))
% plot(squeeze(MK(4,4,:)),'DisplayName','lon')
% plot(squeeze(MK(2,2,:)))
% legend();

% figure
% plot(squeeze(Mk_instant(3,3,:)),'DisplayName','lat')
% hold on
% plot(squeeze(Mk_instant(1,1,:)))
% plot(squeeze(Mk_instant(4,4,:)),'DisplayName','lon')
% plot(squeeze(Mk_instant(2,2,:)))
% legend();

%% 距离辅助导航
result=cell(1,4);
kk=1;
for ii=[1:4,9]
    range=RNG{ii};
    beacon=BCN{ii};
    dr = mydr('init',avp_ref(1,7:9)',[1;1;0.2],ts);

    % % x0=[0.02;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
    % % dx0=[0.01;d2r(1);1/glv.Re;1/glv.Re];
    % 
    % % 初始值的设置也会影响距离辅助的效果
    % % x0=[0.05;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
    % % dx0=[0.01;d2r(1);1/glv.Re;1/glv.Re];
    % 
    % x0=[0;0;0;0];% 初始值
    % dx0=[0.03;d2r(1);1/glv.Re;1/glv.Re];% 初始值不确定性

    kf = myekf('init',0.5, x0, dx0, [0,web,0,0], rngk);
    [avp_range,avp_drdr,kk_1,xkpk]=prealloc(length(avp_ref),10,10,5,9);
    ki=1;
    for i=1:length(compass)
        t=compass(i,end);
        dr=mydr('update',dr,-depther(i),compass(i,3),vxy(i,1:2));
        avp_drdr(i,:)=[dr.avp',t];
        % 以上为航位推算部分
        kf = myekf('fk',kf, dr);
        kf = myekf('algo',kf,'T');
        if mod(t,8)==0
            % 使用水平距离
            if size(beacon,1)==1
                dr.beacon=beacon;
                r=sqrt(range(i)^2-(avp_ref(i,9)-dr.beacon(3))^2);% 测量值
            else
                dr.beacon=beacon(ki,:);
                r=range(ki);
            end
            kf.r_dr=sqrt(RCompu(dr.pos',dr.beacon)^2-(-depther(i)-dr.beacon(3))^2); % 计算值
            kf.yk=r-kf.r_dr;
            kf = myekf('hk',kf,dr,'range');
            kf = myekf('algo',kf,'M');
            % 反馈
            dr.pos(1:2) = dr.pos(1:2)-kf.xk(3:4);
            kf.xk(3:4)=[0;0];
            % dphi=kf.xk(2);
            % kf.xk(2)=0;
            % dkod=kf.xk(1);
            % kf.xk(1)=0;
            dr.avp=[dr.att;dr.vn;dr.pos];
            kk_1(ki,1:4)=kf.xk;
            kk_1(ki,5)=t;
            avp_range(ki,:) = [dr.avp', t];
            xkpk(ki,:)=[kf.xk',diag(kf.Pxk)',t];
            ki=ki+1;
        end
    end
    avp_range(ki:end,:) = [];
    kk_1(ki:end,:) = [];
    xkpk(ki:end,:) = [];
    result{kk}=avp_range;
    kk=kk+1;
    % avp_range(:,7)=avp_range(:,7)-kk_1(:,3);
    % avp_range(:,8)=avp_range(:,8)-kk_1(:,4);
end
%% 误差量化
%% 优化后的性能对比分析代码
% 1. 计算DR误差 (航位推算)
dr_error = RCompu(avp_ref(16:16:end, 7:9), avp_dr(16:16:end, 7:9))';

% 2. 预分配内存并并行处理信标误差
num_methods = 5;  % Beacon1到MovingBeacon
error_bea = cell(1, num_methods);

% 使用逻辑索引替代嵌套循环
ref_times = avp_ref(:, end);  % 参考轨迹时间戳

for i = 1:num_methods
    % 查找匹配的时间点索引
    [~, idx] = ismember(result{i}(:, end), ref_times);
    valid_idx = idx(idx > 0);  % 过滤无效索引
    
    % 计算位置误差
    if ~isempty(valid_idx)
        error_bea{i} = RCompu(avp_ref(valid_idx, 7:9), result{i}(idx > 0, 7:9))';
    else
        error_bea{i} = [];  % 空数组处理
        warning('方法%d无匹配时间点', i);
    end
end

% 3. 创建方法标签和误差集合
methods = {'DR', 'Beacon1', 'Beacon2', 'Beacon3', 'Beacon4', 'MovingBeacon'};
all_errors = [{dr_error}, error_bea];  % 组合所有误差数据

% 4. 结构化的统计计算 (避免冗余计算)
results = struct('Method', methods, ...
                 'MaxError', cell(1,6), ...
                 'MeanError', cell(1,6), ...
                 'RMS', cell(1,6), ...
                 'StdDev', cell(1,6), ...
                 'Median', cell(1,6), ...
                 'P95', cell(1,6));

for i = 1:length(methods)
    if ~isempty(all_errors{i})
        err = all_errors{i};
        results(i).MaxError = max(err);
        results(i).MeanError = mean(err);
        results(i).RMS = sqrt(mean(err.^2));  % 比rms()更高效
        results(i).StdDev = std(err);
        results(i).Median = median(err);
        results(i).P95 = prctile(err, 95);
    end
end

% 5. 计算性能改进率 (仅对信标方法)
dr_mean = results(1).MeanError;
for i = 2:length(methods)
    if ~isempty(all_errors{i})
        results(i).Improvement = (dr_mean - results(i).MeanError) / dr_mean * 100;
    end
end

% 6. 专业化的表格输出
fprintf('\n=== 导航方法性能对比分析 ===\n');
fprintf('%-16s %-10s %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
        '方法', '平均误差', '最大误差', '均方根', '标准差', '中位数', '95%%分位', '改进率%%');
fprintf(repmat('-', 1, 88) + "\n");

for i = 1:length(results)
    if i == 1  % DR基准方法
        fprintf('%-16s %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f %-10s\n', ...
                results(i).Method, ...
                results(i).MeanError, ...
                results(i).MaxError, ...
                results(i).RMS, ...
                results(i).StdDev, ...
                results(i).Median, ...
                results(i).P95, ...
                'N/A');
    else  % 信标方法
        fprintf('%-16s %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f %-10.1f\n', ...
                results(i).Method, ...
                results(i).MeanError, ...
                results(i).MaxError, ...
                results(i).RMS, ...
                results(i).StdDev, ...
                results(i).Median, ...
                results(i).P95, ...
                results(i).Improvement);
    end
end

%%
myfigurestartup(3,3,'paper'),plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(:,7:9)),'m--')
index=[];
for i=1:5
for j=1:length(result{i})
    index(j)=find(avp_ref(:,end)==result{i}(j,end));
end
hold on
plot(result{i}(:,end),RCompu(avp_ref(index,7:9),result{i}(:,7:9)));
end
xygo('t/s','error/m')
axis([0 8800 0 50])
legend('dr','beacon1','beacon2','beacon3','beacon4','moving beacon','Location','northwest')
% print(gcf, 'New Folder\仿真-信标+径向误差.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\仿真-信标+径向误差.png', '-dpng','-r600');

%% 轨迹/误差绘图
RadialError1=RCompu(avp_ref(16:16:end,7:9),avp_range(:,7:9));
myfigurestartup(7,3,'paper'),
subplot 121,trjsee(avp_ref,'2d',avp_dr,avp_range),legend('true trajectory','DR','DR/range')
dot(3,1);
axis equal
subplot 122,
plot(avp_ref(:,end),RadialError,'r')
hold on
plot(avp_range(:,end),RadialError1,'g')
legend('DR','DR/range')
xlim([avp_ref(1,end) avp_ref(end,end)])
xygo('t/s','Error/m')
% print(gcf, 'New Folder\仿真-轨迹+径向误差.png', '-dpng','-r600');
%%
% lonlat(avp_ref,avp_dr,'prese',1,1,{'11'});
% myfigurestartup(5,5,'paper'),
% lon_lat_err(avp_ref,{'range'},avp_dr,{avp_range});
% print(gcf, 'paper\lonlaterr.svg', '-dsvg');

% print(gcf, 'New Folder\仿真-EKF输出.png', '-dpng','-r600');
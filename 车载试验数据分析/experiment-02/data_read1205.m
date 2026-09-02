clear
%% 读取120串口/文件保存AUAX数据，
for i=1:7
    filename2 = ['D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1205-compass\','Disk1_00',num2str(i-1),'.dat'];
    AUAX{i} = read_auax_120(filename2);
    disp("读取120的串口AUAX数据")
    disp('-----------------')
end
%%
i=5;
disp('---------------------文件-------------------')
% AUAX{i}(2,:)=AUAX{i}(2,:)+18;
index2 = conclude_auax11(AUAX{i});% 多出初始化、等待装订、自检等状态字
disp('--------------------------------------------')
%%
AUAX_all=[];
for i=1:7
    AUAX_all=[AUAX_all,AUAX{i}];
end
AUAX_all(:,AUAX_all(6,:)<36.36)=[];
%%
idx_17152=find(AUAX_all(1,:)==430.015|AUAX_all(1,:)==430.01);
% idx_17152 已经提前算出
nSeg = length(idx_17152);
AUAX_cell = cell(nSeg-1,1);   % 存放每一段矩阵
for k = 1 : nSeg-1
    % 切片：idx(k) ~ idx(k+1)-1
    colStart = idx_17152(k);
    colEnd   = idx_17152(k+1) - 1;
    
    AUAX_tmp = AUAX_all(:, colStart : colEnd);
    
    % % 过滤：保留第32行等于17152的列
    keepCol = AUAX_tmp(32,:) == 17152;
    AUAX_tmp = AUAX_tmp(:, keepCol);
    
    AUAX_cell{k} = AUAX_tmp;
    
    % 绘图
    figure; hold on; grid on;
    plot(AUAX_tmp(7,:), AUAX_tmp(6,:),'LineWidth',1);
    plot(AUAX_tmp(7,1),   AUAX_tmp(6,1),'*','MarkerSize',10);
    plot(AUAX_tmp(7,end), AUAX_tmp(6,end),'o','MarkerSize',10);
    legend('trj','start','end','Location','best');
    title(['片段 ',num2str(k)]);
    hold off;
end
%
AUAX_VADIATE = AUAX_cell([2,4],:);
figure
for i=1:2
    subplot(1,2,i),hold on,grid on
    AUAX_tmp=AUAX_VADIATE{i,1};
    plot(AUAX_tmp(7,:), AUAX_tmp(6,:),'LineWidth',1);
    plot(AUAX_tmp(7,1),   AUAX_tmp(6,1),'*','MarkerSize',10);
    plot(AUAX_tmp(7,end), AUAX_tmp(6,end),'o','MarkerSize',10);
end
legend('trj','start','end','Location','best');
%% 1
fid=fopen("D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1205-compass\2025_12_05_15_07_10_连续.txt");
fgets(fid);
DistData_rw=fscanf(fid,'%d:%d:%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',[23,inf]);
fclose(fid);

for i=1:length(DistData_rw)
    [week_number, DistData_rw(24,i)] = bjt_to_utc_week_seconds(2025,12,5,DistData_rw(1,i),DistData_rw(2,i),DistData_rw(3,i),18);
end
%%
DistData_rw(24,:)=DistData_rw(24,:)-18-3;
id1=find(abs(AUAX_VADIATE{1}(2,:)-DistData_rw(24,1))<0.005);
id2=find(abs(AUAX_VADIATE{1}(2,:)-DistData_rw(24,84000))<0.005);
data1=AUAX_VADIATE{1}(:,id1:id2);
angle120=data1(2:3,:);
angel=angle120(2,:);
angel(angel(:)>0)=360-angel(angel(:)>0);
angel(angel(:)<0)=-angel(angel(:)<0);
angle120(2,:)=angel;
angelcompass=DistData_rw([24,4],:);
[x_unique, ~, ic] = unique(angelcompass(1,:));
y_mean = accumarray(ic, angelcompass(2,:), [], @mean);
angelcompass_intep(2,:) = interp1(x_unique, y_mean, angle120(1,:), "linear");
angelcompass_intep(1,:)=angle120(1,:);
%%
myfigurestartup(10,5,'prese');
subplot 121
plot(angle120(1,:),angle120(2,:))
hold on
plot(angelcompass_intep(1,:),angelcompass_intep(2,:))
legend('120','罗盘')
ylabel('角度/°')
xlabel('时间/s')
subplot 122
dangle=angelcompass_intep(2,:)-angle120(2,:);
% dangle(dangle<-100)=dangle(dangle<-100)+360;
plot(angelcompass_intep(1,:),dangle)
ylabel('角度差')
xlabel('时间/s')
%% 2
fid=fopen("D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1205-compass\2025_12_05_16_25_03_连续.txt");
fgets(fid);
DistData_rw2=fscanf(fid,'%d:%d:%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',[23,inf]);
fclose(fid);
for i=1:length(DistData_rw2)
    [week_number, DistData_rw2(24,i)] =bjt_to_utc_week_seconds(2025,12,5,DistData_rw2(1,i),DistData_rw2(2,i),DistData_rw2(3,i),0);
end
DistData_rw2(24,:)=DistData_rw2(24,:)-21;
%%
id1=find(abs(AUAX_VADIATE{2}(2,:)-DistData_rw2(24,8000))<0.005);
id2=find(abs(AUAX_VADIATE{2}(2,:)-DistData_rw2(24,end))<0.005);
data1=AUAX_VADIATE{2}(:,id1:id2);
angle120=data1(2:3,:);
angel=angle120(2,:);
angel(angel(:)>0)=360-angel(angel(:)>0);
angel(angel(:)<0)=-angel(angel(:)<0);
angle120(2,:)=angel;
angelcompass=DistData_rw2([24,4],:);
[x_unique, ~, ic] = unique(angelcompass(1,:));
y_mean = accumarray(ic, angelcompass(2,:), [], @mean);
angelcompass_intep2(2,:) = interp1(x_unique, y_mean, angle120(1,:), "linear");
angelcompass_intep2(1,:)=angle120(1,:);
%%
myfigurestartup(10,5,'prese');
subplot 121
plot(angle120(1,:),angle120(2,:))
hold on
plot(angelcompass_intep2(1,:),angelcompass_intep2(2,:))
legend('120','罗盘')
ylabel('角度/°')
xlabel('时间/s')
subplot 122
dangle=angelcompass_intep2(2,:)-angle120(2,:);
% dangle(dangle<-100)=dangle(dangle<-100)+360;
plot(angelcompass_intep2(1,:),dangle)
ylabel('角度差')
xlabel('时间/s')

%% 3
fid=fopen("D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1205-compass\2025_12_05_17_21_45_连续.txt");
fgets(fid);
DistData_rw3=fscanf(fid,'%d:%d:%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',[23,inf]);
fclose(fid);
for i=1:length(DistData_rw3)
    [week_number, DistData_rw3(24,i)] =bjt_to_utc_week_seconds(2025,12,5,DistData_rw3(1,i),DistData_rw3(2,i),DistData_rw3(3,i),18);
end
DistData_rw3(24,:)=DistData_rw3(24,:)-21;
%%
id1=find(abs(AUAX{6}(2,:)-DistData_rw3(24,1))<0.005);
id2=find(abs(AUAX{6}(2,:)-DistData_rw3(24,57100))<0.005);
data1=AUAX{6}(:,id1:id2);
angle120 =data1(2:3,:);
angel=angle120(2,:);
angel(angel(:)>0)=360-angel(angel(:)>0);
angel(angel(:)<0)=-angel(angel(:)<0);
angle120(2,:)=angel;
angelcompass=DistData_rw3([24,4],:);
[x_unique, ~, ic] = unique(angelcompass(1,:));
y_mean = accumarray(ic, angelcompass(2,:), [], @mean);
angelcompass_intep3(2,:) = interp1(x_unique, y_mean, angle120(1,:), "linear");
angelcompass_intep3(1,:) = angle120(1,:);
%%
myfigurestartup(10,5,'prese');
subplot 121
plot(angle120(1,:),angle120(2,:))
hold on
plot(angelcompass_intep3(1,:),angelcompass_intep3(2,:))
legend('120','罗盘')
ylabel('角度/°')
xlabel('时间/s')
subplot 122
dangle=angelcompass_intep3(2,:)-angle120(2,:);
% dangle(dangle<-100)=dangle(dangle<-100)+360;
plot(angelcompass_intep3(1,:),dangle)
ylabel('角度差')
xlabel('时间/s')

%%
function index= conclude_auax11(AUAX)
% -------AUAX数据概述---------
unique_values1 = unique(AUAX(end-2,:));
% 为数据创建状态字符串单元格数组
state1 = cell(1, length(unique_values1));
for i = 1:length(unique_values1)
    state1{i} = dec2hex(unique_values1(i));
end
% 动态创建格式字符串
formatStr1 = repmat('%s,', 1, length(unique_values1));
formatStr1 = formatStr1(1:end-1); % 移除最后一个逗号
fprintf('状态字有：');
fprintf([formatStr1 '\n'], state1{:});
[starttime,endtime] = week_sec2utc(AUAX(1:2,end),2395);
fprintf('起始时间: %s\n', starttime);
fprintf('结束时间: %s\n', endtime);

% 分开状态
index.useless=find(AUAX(end-2,:)==hex2dec('0'));
index.coarse=find(AUAX(end-2,:)==hex2dec('5840'));
index.fine=find(AUAX(end-2,:)==hex2dec('5820'));
index.nav=find(AUAX(end-2,:)==hex2dec('4300'));

leap_sec = 0;
beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2395, AUAX(2,index.fine(1)), leap_sec);
fprintf('精对准开始时间为: %s \n',beijing_time_precise)

beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2395, AUAX(2,index.fine(end)), leap_sec);
fprintf('精对准结束时间为: %s \n',beijing_time_precise)

beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2395, AUAX(2,index.nav(1)), leap_sec);
fprintf('导航开始时间为: %s \n',beijing_time_precise)

beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2395, AUAX(2,index.nav(end)), leap_sec);
fprintf('导航结束时间为: %s \n',beijing_time_precise)
end


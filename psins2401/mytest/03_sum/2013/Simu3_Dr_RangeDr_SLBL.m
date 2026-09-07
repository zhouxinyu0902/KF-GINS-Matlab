clear
load('data_1\data_dr.mat')
glvs
ts=trj.ts;

depther=-avp_ref(:,9)+normrnd(0,0.2,size(avp_ref(:,9)));
%% 仿真条件绘图
myfigurestartup(5,5,'paper')
insplot(trj.avp(:,[1:6,end]),'av')
% print(gcf, 'New Folder\仿真-姿态和速度.png', '-dpng','-r600');

myfigurestartup(7,3,'paper')
subplot 121,insplot(trj.avp(:,[7:9,end]),'p')
subplot 122,insplot(trj.avp(:,[7:8,end]),'l')
% print(gcf, 'New Folder\仿真-位移+轨迹.png', '-dpng','-r600');
%% 航位推算的误差绘图
figure
plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(:,7:9)),'m')
%% 信标位置和距离
rngk=5;rngc=0;
[RNG,BCN]=beacon_gen(avp_ref(1,7:9)',5,0.2,avp_ref,8,0);
% %% 信标+轨迹绘图
myfigurestartup(3,3,'paper')
dxyz = pos2dxyz([avp_ref(:,7:8),avp_ref(:,9)*0]);
dxyzship = pos2dxyz([BCN{9}(:,1),BCN{9}(:,2),BCN{9}(:,2)*0],avp_ref(1,7:9)');
plot(0, 0, 'rp');
hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
hold on, plot(dxyzship(1,1), dxyzship(1,2),'*');
hold on, plot(dxyzship(:,1), dxyzship(:,2));
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
% print(gcf, 'New Folder\仿真-信标+轨迹.png', '-dpng','-r600');
%% 距离辅助（单个信标）
ii=1;
range=RNG{ii};
beacon=BCN{ii};
dr = mydr('init',avp_ref(1,7:9)',[0.1;0.1;0.1],ts);

% 初始值的设置也会影响距离辅助的效果
% x0=[0.05;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
% dx0=[0.01;d2r(1);1/glv.Re;1/glv.Re];

x0=[0;0;0;0];% 初始值
dx0=[0.03;d2r(5);1/glv.Re;1/glv.Re];% 初始值不确定性

kf = myekf('init',0.5, x0, dx0, [0,web,0,0], rngk);
[avp_range,avp_drrange,kk_1,xkpk]=prealloc(length(avp_ref),10,10,5,9);
ki=1;
for i=1:length(compass)
    t=compass(i,end);
    dr=mydr('update',dr,-depther(i),compass(i,3),vxy(i,1:2));
    avp_drrange(i,:)=[dr.avp',t];
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
% 单个信标绘图 径向误差+估计状态
myfigurestartup(3,3,'paper'),
plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(:,7:9)),'m')
% hold on
% plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_drrange(:,7:9)),'m')
hold on
plot(avp_range(:,end),RCompu(avp_ref(16:16:end,7:9),avp_range(:,7:9)));

myfigurestartup(5,5,'paper')
xk_plot(0.05,d2r(0.5),dr_err,{'propa'},{xkpk(:,[1:4,end])},0)
%%% 以上是估计出误差最后再反馈的方式
%%% 研究每一个距离来临时后，将误差反馈，刻度系数反馈
%% 距离辅助（单个信标）
ii=1;
range=RNG{ii};
beacon=BCN{ii};
dr = mydr('init',avp_ref(1,7:9)',[0.1;0.1;0.1],ts);

% 初始值的设置也会影响距离辅助的效果
% x0=[0.05;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
% dx0=[0.01;d2r(1);1/glv.Re;1/glv.Re];

x0=[0;0;0;0];% 初始值
dx0=[0.03;d2r(1);1/glv.Re;1/glv.Re];% 初始值不确定性

dkod=0;
dphi=0;

kf = myekf('init',0.5, x0, dx0, [0,0,web,0], rngk);
[avp_range,avp_drrange,kk_1,xkpk]=prealloc(length(avp_ref),10,10,5,9);
ki=1;
for i=1:length(compass)
    t=compass(i,end);
    % 补偿
    % compass(i,3)=compass(i,3)-dphi;
    % vxy(i,1:2)=(1+dkod)*vxy(i,1:2);
    dr=mydr('update',dr,-depther(i),compass(i,3),vxy(i,1:2));
    avp_drrange(i,:)=[dr.avp',t];
    % 以上为航位推算部分
    kf = myekf('fk',kf, dr);
    kf = myekf('algo',kf,'T');
    if mod(t,8)==0
        % 使用水平距离
        % if size(beacon,1)==1
            dr.beacon=beacon;
            r=sqrt(range(i)^2-(avp_ref(i,9)-dr.beacon(3))^2);% 测量值
        % else
        %     dr.beacon=beacon(ki,:);
        %     r=range(ki);
        % end
        kf.r_dr=sqrt(RCompu(dr.pos',dr.beacon)^2-(-depther(i)-dr.beacon(3))^2); % 计算值
        kf.yk=r-kf.r_dr;
        kf = myekf('hk',kf,dr,'range');
        kf = myekf('algo',kf,'M');
        kk_1(ki,1:4)=kf.xk;
        kk_1(ki,5)=t;
        % 反馈
        dr.pos(1:2) = dr.pos(1:2)-kf.xk(3:4);
        kf.xk(3:4)=[0;0];
        dphi=kf.xk(2);
        % kf.xk(2)=0;
        dkod=kf.xk(1);
        % kf.xk(1)=0;
        dr.avp=[dr.att;dr.vn;dr.pos];
        avp_range(ki,:) = [dr.avp', t];
        xkpk(ki,:)=[kf.xk',diag(kf.Pxk)',t];
        ki=ki+1;
    end
end
avp_range(ki:end,:) = [];
kk_1(ki:end,:) = [];
xkpk(ki:end,:) = [];
%%
myfigurestartup(5,5,'paper')
plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(:,7:9)),'m')
hold on
plot(avp_range(:,end),RCompu(avp_ref(16:16:end,7:9),avp_range(:,7:9)));
%% 查看前四个位置点和信标位置
figure
for i=1:10:40
plot(avp_range(i,8),avp_range(i,7),'*')
hold on
end
plot(beacon(2),beacon(1),'.')
%% 距离辅助导航
for ii=[1,2,3,4,9]
    range=RNG{ii};
    beacon=BCN{ii};
    dr = mydr('init',avp_ref(1,7:9)',[0.1;0.1;0.1],ts);
    x0=[0.02;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
    dx0=[0.01;d2r(1);1/glv.Re;1/glv.Re];

    kf = myekf('init',0.5, x0, dx0, [0,web,0,0], rngk);
    [avp_range,avp_dr,kk_1,xkpk]=prealloc(length(avp_ref),10,10,5,9);
    ki=1;
    for i=1:length(compass)
        t=compass(i,end);
        dr=mydr('update',dr,-depther(i),compass(i,3),vxy(i,1:2));
        avp_dr(i,:)=[dr.avp',t];
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
    if ii==1
        myfigurestartup(3,3,'paper'),plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(:,7:9)),'m--')
    end
    % 连续画图看位置误差
    index=[];
    for j=1:length(avp_range)
        index(j)=find(avp_ref(:,end)==avp_range(j,end));
    end
    hold on
    plot(avp_range(:,end),RCompu(avp_ref(index,7:9),avp_range(:,7:9)));
end
xygo('t/s','error/m')
axis([0 8800 0 50])
legend('dr','beacon1','beacon2','beacon3','beacon4','moving beacon','Location','northwest')
% print(gcf, 'New Folder\仿真-信标+径向误差.png', '-dpng','-r600');
%% 轨迹/误差绘图
RadialError1=RCompu(avp_ref(80:80:end,7:9),avp_range(:,7:9));
myfigurestartup(7,3,'paper'),
subplot 121,trjsee(avp_ref,'2d',avp_dr,avp_range),legend('true trajectory','DR','DR/range')
dot(3);
axis equal
subplot 122,
plot(avp_ref(:,end),RadialError,'r')
hold on
plot(avp_range(:,end),RadialError1,'g')
legend('DR','DR/range')
xlim([avp_ref(1,end) avp_ref(end,end)])
xygo('t/s','Error/m')
print(gcf, 'New Folder\仿真-轨迹+径向误差.png', '-dpng','-r600');
%%
% lonlat(avp_ref,avp_dr,'prese',1,1,{'11'});
% myfigurestartup(5,5,'paper'),
% lon_lat_err(avp_ref,{'range'},avp_dr,{avp_range});
% print(gcf, 'paper\lonlaterr.svg', '-dsvg');
dr_err=avpcmp(avp_dr,avp_ref);
myfigurestartup(5,5,'paper'),xk_plot(0.05,0.01,dr_err,'range-aided',{kk_1},0)
print(gcf, 'New Folder\仿真-EKF输出.png', '-dpng','-r600');
clear

% path = "D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_lawnmower_single_side";
path = "D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_line_N_single_side";
pva_ref = importdata(fullfile(path,"reference.txt"));
avp_ref = pvaNED2ENU(pva_ref);
compass = importdata(fullfile(path,"compass.txt"));
Range = importdata(fullfile(path,"beacon.txt"));
Range = Range(8:8:end,:);
depther = importdata(fullfile(path,"depth.txt"));
depther = depther(:,2);
dvl = importdata(fullfile(path,"dvl.txt"));
vxy = dvl(:,2:3);
%
rngk = 5;
depthstd = 0.4;
glvs
ts=0.5;
rng(1)
%% 初始值的设置也会影响距离辅助的效果
% x0=[0.05;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
% dx0=[0.01;d2r(1);1/glv.Re;1/glv.Re];

x0=[0;0;0;0];% 初始值
dx0=[0.05;d2r(0.5);1/glv.Re;1/glv.Re];% 初始值不确定性

% 距离辅助
dr = [];
range = Range(:,[1,3]);
beacon = Range(1,4:6);
dr = mydr('init',avp_ref(1,7:9)',[1;1;0.2],ts);
kf = myekf('init',0.5, x0, dx0, [0,d2r(0.2),0,0], rngk);
[avp_range,avp_dr1,kk_1,xkpk] = prealloc(length(avp_ref),10,10,5,9);
ki=1;
for i=1:length(compass)
    t = compass(i,1);
    dr = mydr('update',dr,-depther(i),compass(i,4),vxy(i,1:2));
    avp_dr1(i,:)=[dr.avp',t];
    % 以上为航位推算部分
    kf = myekf('fk',kf, dr);
    kf = myekf('algo',kf,'T');
    if range(ki,1) == t
        % 使用水平距离
        dr.beacon = beacon;
        r = range(ki,2);
        kf.r_dr = sqrt(RCompu(dr.pos',dr.beacon)^2-(-depther(i)-dr.beacon(3))^2); % 计算值
        kf.yk =  kf.r_dr - r;
        kf = myekf('hk',kf,dr,'range');
        kf = myekf('algo',kf,'M');
        res(ki)=kf.yk ;
        kk_1(ki,1:4) = kf.xk;
        kk_1(ki,5) = t;
        avp_range(ki,:) = [dr.avp', t];
        ki=ki+1;
        if ki >length(range)
            break;
        end
    end
end
avp_range(ki:end,:) = [];
avp_dr1(i:end,:) = [];
kk_1(ki:end,:) = [];
xkpk(ki:end,:) = [];
avp_range(:,7)=avp_range(:,7)-kk_1(:,3);
avp_range(:,8)=avp_range(:,8)-kk_1(:,4);
%% 单个信标绘图 径向误差+估计状态
trjsee(avp_ref,'2d',avp_dr1,avp_range),legend('true trajectory','DR','DR/Range')

% axis equal
myfigurestartup(5,3,'paper');
RadialError = RCompu(avp_ref(1:length(avp_dr1),7:9),avp_dr1(:,7:9));
RadialError_range = RCompu(avp_ref(16:16:end,7:9),avp_range(:,7:9));
plot(avp_ref(1:length(RadialError),end),RadialError)
hold on
plot(avp_ref(16:16:end,end),RadialError_range)
legend('DR','DR/Range')
xlim([avp_ref(1,end) avp_ref(end,end)])
xygo('t/s','Error/m')

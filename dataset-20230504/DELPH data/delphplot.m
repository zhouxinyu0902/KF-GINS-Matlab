clear
filepath='F:\4_program\DELPH_INS\DELPH\p1\old-version\ins_dvl_4lbl.xpf.txt';
fid=fopen(filepath,'rt'); 
for i=1:30
fgets(fid);
end
ins_dvl_4lbl=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',[30,inf]);
ins_dvl_4lbl(31,:)=ins_dvl_4lbl(4,:)*3600+ins_dvl_4lbl(5,:)*60+ins_dvl_4lbl(6,:)+ins_dvl_4lbl(7,:)/10000;
ins_dvl_4lbl(1:8,:)=[];
%% 绘图定位结果
load('psins2401\mytest\sum\2023\matData\USBL.mat')
load('psins2401\mytest\sum\2023\matData\LBL.mat')
load('psins2401\mytest\sum\2023\matData\PHINS-new.mat')
position=ins_dvl_4lbl(1:3,:);

myfigurestartup(5,5,'prese'),plot(position(2,:),position(1,:))
hold on
plot(Ins_Nav(2,:),Ins_Nav(1,:))
plotlbl(POS_auv);
legend('ins+dvl+4lbl','real time navigation','LBL')
xygo('lat','lon')
plotbeacon4(bcn);

%% 位置绘图
posstd=ins_dvl_4lbl(4:6,:);
northesatcov=ins_dvl_4lbl(7,:);
myfigurestartup(12,5,'prese'),plotstd(posstd)

%% 姿态绘图,deg为单位
att=ins_dvl_4lbl(8:10,:);
myfigurestartup(12,5,'prese'),plotatt(att)
attitude_ref=[att;ins_dvl_4lbl(end,:)];

attstd=ins_dvl_4lbl(11:13,:);
myfigurestartup(12,5,'prese'),plotstd(attstd)

%% 速度绘图
speed_n=ins_dvl_4lbl(14:16,:);
myfigurestartup(12,5,'prese'),plotstd(speed_n)

vel_nstd=ins_dvl_4lbl(17:19,:);
myfigurestartup(12,5,'prese'),plotstd(vel_nstd)

speed_b=ins_dvl_4lbl(20:23,:);
myfigurestartup(12,5,'prese'),plotstd(speed_b)

speed_n_ref=[speed_n;ins_dvl_4lbl(end,:)];
speed_b_ref=[speed_b;ins_dvl_4lbl(end,:)];

save 'DELPH data'\att_vel.mat attitude_ref speed_n_ref speed_b_ref
%% 函数
function plotbeacon4(bcn)
% 画出信标的位置，单位为嘟
for i=1:4
    hold on
    plot(bcn{i}(2),bcn{i}(1),'*')
end
end
function plotlbl(posauv)
    hold on
    plot(posauv(9,:),posauv(8,:))
end
function plotatt(att)
str={'heading','roll','pitch'};
for i=1:3
    subplot(1,3,i)
    plot(att(i,:))
    title(str{i})
    grid on
    % ConvertXAxisTime;
end
end
function plotstd(std)
for i=1:3
    subplot(1,3,i)
    plot(std(i,:))
    grid on
    % ConvertXAxisTime;
end
end
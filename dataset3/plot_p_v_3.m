clear
imu=importdata("dataset3\ADIS16465.txt");
gnss=importdata("dataset3\GNSS-POS.txt");
ref=importdata("dataset3\truth.nav");
%%
close all
glvs
myfigurestartup(10,5,'prese')
gnss_rrm(:,1:2)=gnss(:,2:3)/180*pi;
gnss_rrm(:,3)=gnss(:,4);
dxyz=pos2dxyz(gnss_rrm(:,1:3),gnss_rrm(1,1:3)');
subplot 121,plot(dxyz(:,1),dxyz(:,2))
tsum=gnss(end,1)-gnss(1,1)
subplot 122,plot(ref(:,2),ref(:,6),ref(:,2),ref(:,7))
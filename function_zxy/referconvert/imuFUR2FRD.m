function IMUFRD = imuFUR2FRD(IMUFUR)
% 将测量值前上右转为kfgins数据前右上
% 将角速度（°/s）和加速度（m/s^2）转为角度增量和速度增量
IMUFRD=IMUFUR(:,1:7);
IMUFRD(:,1)=IMUFUR(:,8);
IMUFRD(:,2)=IMUFUR(:,1);
IMUFRD(:,3)=IMUFUR(:,3);
IMUFRD(:,4)=-IMUFUR(:,2);
IMUFRD(:,5)=IMUFUR(:,4);
IMUFRD(:,6)=IMUFUR(:,6);
IMUFRD(:,7)=-IMUFUR(:,5);
IMUFRD(:,2:4)=IMUFRD(:,2:4)*0.01/180*pi;
IMUFRD(:,5:7)=IMUFRD(:,5:7)*0.01;
end

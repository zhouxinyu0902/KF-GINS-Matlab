function IMUFRD = imuRFU2FRD(IMURFU)
% 将psins数据右前上转为kfgins测量前右下，并保存
IMUFRD(:,1)=IMURFU(:,7);
IMUFRD(:,2)=IMURFU(:,2);
IMUFRD(:,3)=IMURFU(:,1);
IMUFRD(:,4)=-IMURFU(:,3);
IMUFRD(:,5)=IMURFU(:,5);
IMUFRD(:,6)=IMURFU(:,4);
IMUFRD(:,7)=-IMURFU(:,6);
end


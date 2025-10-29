function IMURFU = imuFRD2RFU(IMUFRD)
% 将kfgins测量前右下转为psins数据右前上
IMURFU(:,1)=IMUFRD(:,3);
IMURFU(:,2)=IMUFRD(:,2);
IMURFU(:,3)=-IMUFRD(:,4);
IMURFU(:,4)=IMUFRD(:,6);
IMURFU(:,5)=IMUFRD(:,5);
IMURFU(:,6)=-IMUFRD(:,7);
IMURFU(:,7)=IMUFRD(:,1);
end


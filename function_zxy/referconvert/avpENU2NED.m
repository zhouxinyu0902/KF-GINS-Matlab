function pva_ref = avpENU2NED(avp)
% 将仿真的参考值ENU转到NED下
% pva=[0,t,l,l,h,v,v,v,r,p,y]
% avp=[p,r,y,v,v,v,l,l,h,t];
pva_ref(:,3:4)=r2d(avp(:,7:8));
pva_ref(:,5)=avp(:,9);

pva_ref(:,6)=avp(:,5);
pva_ref(:,7)=avp(:,4);
pva_ref(:,8)=-avp(:,6);

pva_ref(:,9)=r2d(avp(:,2));
pva_ref(:,10)=r2d(avp(:,1));
pva_ref(:,11)=r2d(yawcvt(avp(:,3),"cc180c360"));

pva_ref(:,2)=avp(:,end);
% 第一列全为0
pva_ref(:,1)=zeros(size(avp(:,end)));
end


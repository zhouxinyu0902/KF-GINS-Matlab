function avp = pvaNED2ENU(pva)

avp(:,2)=pva(:,9)/180*pi;
avp(:,1)=pva(:,10)/180*pi;
avp(:,3)=(yawcvt(pva(:,11)/180*pi,"c360cc180"));

avp(:,5)=pva(:,6);
avp(:,4)=pva(:,7);
avp(:,6)=-pva(:,8);

avp(:,7:8)=pva(:,3:4)/180*pi;
avp(:,9)=pva(:,5);
avp(:,10)=pva(:,2);

end
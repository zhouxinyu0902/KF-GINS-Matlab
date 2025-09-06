function avp = pvaNED2ENU(pva)

avp(:,2)=d2r(pva(:,9));
avp(:,1)=d2r(pva(:,10));
avp(:,3)=(yawcvt(d2r(pva(:,11)),"c360cc180"));

avp(:,5)=pva(:,6);
avp(:,4)=pva(:,7);
avp(:,6)=-pva(:,8);

avp(:,7:8)=d2r(pva(:,3:4));
avp(:,9)=pva(:,5);
avp(:,10)=pva(:,2);

end
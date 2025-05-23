function plot_xk(xkpath,stdpath,navpath,refpath)
param=Param();

nav=importdata(navpath);
ref=importdata(refpath);
id=ismember(ref(:,2),nav(:,2));
ref=ref(id,:);
err(:,1)=ref(:,2);
err(:,2:10)=nav(:,3:11)-ref(:,3:11);
err(:,2:3)=err(:,2:3)*param.D2R;
err(:,8:10)=err(:,8:10)*param.D2R;
err(err(:,10)<-pi,10)=err(err(:,10)<-pi,10)+2*pi;

[rm,rn]=getRmRn(ref(1,3)*param.D2R,param);
err(:,2)=err(:,2)*(rm+ref(1,5));
err(:,3)=err(:,3)*cos(ref(1,3))*rn;
err(:,8:9)=err(:,8:9)*param.R2D;

xk=importdata(xkpath);
xk(:,2)=xk(:,2)*(rm+ref(1,5));
xk(:,3)=xk(:,3)*cos(ref(1,3))*rn;
xk(:,8:9)=xk(:,8:9)*param.R2D;

myfigurestartup(10,10,'prese')
subplot(3,3,1)
plot(err(:,1),err(:,2),xk(:,1),xk(:,2))
legend('dlat','estimated dlat')
subplot(3,3,2)
plot(err(:,1),err(:,3),xk(:,1),xk(:,3))
legend('dlon','estimated dlon')
subplot(3,3,3)
plot(err(:,1),err(:,4),xk(:,1),xk(:,4))
legend('dh','estimated dh')

% myfigurestartup(10,3,'prese')
subplot(3,3,4)
plot(err(:,1),err(:,5),xk(:,1),xk(:,5))
legend('dVN','estimated dVN')
subplot(3,3,5)
plot(err(:,1),err(:,6),xk(:,1),xk(:,6))
legend('dVE','estimated dVE')
subplot(3,3,6)
plot(err(:,1),err(:,7),xk(:,1),xk(:,7))
legend('dVD','estimated dVD')

% myfigurestartup(10,3,'prese')
subplot(3,3,7)
plot(err(:,1),err(:,8),xk(:,1),xk(:,8))
legend('droll','estimated droll')
subplot(3,3,8)
plot(err(:,1),err(:,9),xk(:,1),xk(:,9))
legend('dpitch','estimated dpitch')
subplot(3,3,9)
plot(err(:,1),err(:,10),xk(:,1),xk(:,10))
legend('dheading','estimated dheading')

std=importdata(stdpath);
myfigurestartup(7,7,'prese')
subplot 221,plot(xk(:,1),xk(:,11:13)* param.R2D *3600)
title('estimated eb/(deg/h)')
subplot 222,plot(xk(:,1),xk(:,14:16)* 1e5)
title('estimated db/(mGal)')
subplot 223,plot(std(:,1),std(:,11:13))
title('diag(PX)')
subplot 224,plot(std(:,1),std(:,14:16))
title('diag(PX)')


myfigurestartup(10,3,'prese')
subplot 131,plot(std(:,1),std(:,2:4))
title('diag(PX) position')
subplot 132,plot(std(:,1),std(:,5:7))
title('diag(PX) velocity')
subplot 133,plot(std(:,1),std(:,8:10))
title('diag(PX) attitude')
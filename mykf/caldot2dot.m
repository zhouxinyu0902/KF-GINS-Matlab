function [SlantR,HorizR]=caldot2dot(dot1,dot2)
% dot1，dot2，单位为deg,deg,m
param = Param();
sub=dot1-dot2;
sub(:,1:2)=sub(:,1:2)*param.D2R;

[rm, rn] = getRmRn(dot1(1) * param.D2R, param);
h = dot1(3);
DR = diag([rm + h, (rn + h)*cos(dot1(1) * param.D2R), -1]);

delta_pos =( DR * sub')';
SlantR=sqrt(sum(delta_pos(:,1:3).^2,2));
HorizR=sqrt(SlantR.^2-delta_pos(:,3).^2);

% pos1=[d2r(15),d2r(115),20];
% pos2=[d2r(15.1),d2r(115.2),20];
function phi=poscalyaw(pos1,pos2)
% pos1=[d2r(40),d2r(-75),20];
% pos2=[d2r(40.01),d2r(-74.99),20];

param=Param();
[rm, rn] = getRmRn(pos1(1), param);
dlat=pos2(1)-pos1(1);
dlon=pos2(2)-pos1(2);
phi=atan2((rn+pos1(3))*cos(pos1(1))*dlon,(rm++pos1(3))*dlat);
if dlon<0 % 四象限&&三象限
    phi=2*pi+phi;
end  
r2d(phi);
end
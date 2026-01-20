function [A,C,M]=calculate_Matrix(navstate,Rangedata)
global rangstd
param = Param();
bcn = Rangedata(4:6)';
dt=0.1;
[rm, rn] = getRmRn(bcn(1) , param);
h = bcn(3);
DR = diag([rm + h, (rn + h)*cos(bcn(1)), -1]);
dpos =(navstate.pos-bcn);
delta_pos =( DR * (navstate.pos-bcn))';
SlantR=sqrt(sum(delta_pos(:,1:3).^2,2));

K=(Rangedata(3)-SlantR)/(rangstd^2*SlantR);
M=K*(dt^3);

cbn=navstate.cbn';
C=cbn*(DR^-1)'*dpos;
A=K*(cbn*dt^2+cbn*skew_symmetric(navstate.wnin)*dt^3)*(DR^-1)'*dpos;
end
% 辅助函数：计算反对称矩阵 [v x]
function S = skew_symmetric(v)
S = [ 0,    -v(3),  v(2);
    v(3),  0,    -v(1);
    -v(2),  v(1),  0  ];
end

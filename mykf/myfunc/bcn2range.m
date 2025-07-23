function range=bcn2range(truth,bcn)
% 输入为度数
param = Param();
dif(:,1) = truth(:,2);
truth(:,3:4) = truth(:,3:4)* param.D2R;
bcn(:,1:2) = bcn(:,1:2)* param.D2R;

dif(:,2:4)= truth(:,3:5)-bcn;
first_blh = truth(1, 3:5);
[rm, rn] = getRmRn(first_blh(1), param);
h = first_blh(3);
DR = diag([rm + h, (rn + h)*cos(first_blh(1)), 1]);

for i = 1:size(dif, 1)
    delta_pos = DR * (dif(i, 2:4)');
    dif(i, 2:4) = delta_pos';
end


SlantRange=sqrt(sum(dif(:,2:4).^2,2));
HorizonalRange=sqrt(SlantRange.^2-dif(:,4).^2);
myfigurestartup(10,4,'prese')
subplot 121
plot(dif(:,1),SlantRange);
title('SlantRange');
xlabel('Time[s]');
ylabel('Error[deg]');
grid("on");

subplot 122
plot(dif(:,1),HorizonalRange);
title('HorizonalRange');
xlabel('Time[s]');
ylabel('Error[deg]');
grid("on");
if size(bcn,1)==1
    bcn=ones(size(truth(:,1:3))).*bcn;
end
range=[truth(:,2),SlantRange,HorizonalRange,bcn];
end
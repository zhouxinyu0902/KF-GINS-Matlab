function [range,dll] = RCompu(dot1,dot2)
%1.计算两个点的距离
%2.计算很多个点和一个点的距离
%点均以行向量的形式存在，单个点放后面
glvs
[m,n]=size(dot1);
[m1,~]=size(dot2);
[range,dll]=prealloc(n,1,2);
if m>1&&m1==1
    for i=1:length(dot1)
        [RMh, clRNh] = RMRN(dot1(i,:));
        dllh=(dot1(i,:)-dot2)*diag([RMh;clRNh;1]);
        range(i)=sum(dllh.^2).^0.5;
        dll(i,:)=dllh(1:2);
    end
elseif m==1&&m1==1
    [RMh, clRNh] = RMRN(dot1);
    dllh=(dot1-dot2)*diag([RMh;clRNh;1]);
    range=sum(dllh.^2).^0.5;
    
elseif m>1&&m1>1&&m==m1
    for i=1:m
        [RMh, clRNh] = RMRN(dot1(i,:));
        dllh=(dot1(i,:)-dot2(i,:))*diag([RMh;clRNh;1]);
        range(i)=sum(dllh.^2).^0.5;
        dll(i,:)=dllh(1:2);
    end
else
    disp('zxy:please input row vector!!!');
end
end


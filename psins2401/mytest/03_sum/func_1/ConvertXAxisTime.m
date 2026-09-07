%把当前图片的横坐标轴由以秒为单位转变为以时分秒为单位时间
%由于采用了手动的坐标显示，所以当缩放图像时，坐标显示不会自动调整，需要再次调用本函数。
function ConvertXAxisTime()
h=gca;  %获得当前图像的坐标轴句柄
x=get(h); %获得当前图像的坐标轴参数
pos = get(gcf,'Position');
if pos(3) < (length(x.XTick)-1)*64  %显示时间需要的最小宽度
    pos(3) = (length(x.XTick)-1)*64;
    set(gcf,'Position',pos);
end
len = x.XTick(end) - x.XTick(1);
for k=1:length(x.XTick)
    hh = floor(x.XTick(k)/3600);
    mm = floor( (x.XTick(k)-hh*3600)/60);
    ss = x.XTick(k)-hh*3600-mm*60;
    hh = mod(hh,24);
    XTickLabel{k}=sprintf('%2d:%02d:%02.0f',hh,mm,ss);
end
set(h,'XTickLabel',XTickLabel);%,'XTick',x.XTick);
% xlabel('时间');


%把当前图片的坐标轴由以分为单位的数值转换为度和分
function ConvertAxisLonLat()
pos = get(gcf,'Position');
if pos(3) < 950  %显示经度需要的最小宽度
    kn = round(pos(4)*950/pos(3));
    pos(2) = pos(2)-(kn-pos(4));
    pos(3) = 950;
    pos(4) = kn;
    set(gcf,'Position',pos);
end

h=gca;  %获得当前图像的坐标轴句柄
x=get(h); %获得当前图像的坐标轴参数
for k=1:length(x.XTick)
    if x.XTick(k)>=0
        s1 = 'E';
    else
        s1 = 'W';
        x.XTick(k) = -x.XTick(k);
    end
    d = floor(x.XTick(k)/60);
    m = x.XTick(k)-d*60;
    XTickLabel{k}=sprintf('%d°%.2f'' %c',d,m,s1);
end
set(h,'XTickLabel',XTickLabel);%,'XTick',x.XTick);

for k=1:length(x.YTick)
    if x.YTick(k)>=0
        s2 = 'N';
    else
        s2 = 'S';
        x.YTick(k) = -x.YTick(k);
    end
    d = floor(x.YTick(k)/60);
    m = x.YTick(k)-d*60;
    YTickLabel{k}=sprintf('%d°%.2f'' %c',d,m,s2);
end
set(h,'YTickLabel',YTickLabel);%,'YTick',x.YTick);
grid on
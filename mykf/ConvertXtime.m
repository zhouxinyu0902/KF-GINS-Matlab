function ConvertXtime()
ax=gca;
x=get(ax);  %获得当前图像的坐标轴句柄
% for k=1:length(x.XTick)
%     hh = floor(x.XTick(k)/3600);
%     mm = floor( (x.XTick(k)-hh*3600)/60);
%     ss = x.XTick(k)-hh*3600-mm*60;
%     hh = mod(hh,24);
%     XTickLabel{k}=sprintf('%2d:%02d:%02.0f',hh,mm,ss);
% end
XTickLabel=x.XTick-x.XTick(1);
set(ax,'XTickLabel',XTickLabel);
xlim([ax.XLim(1) ax.XLim(2)]);
end


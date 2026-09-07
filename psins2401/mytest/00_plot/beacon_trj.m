function beacon_trj(avp,leg,RNG,BCN)
% 画出信标和载体的纬经度对比，距离信息对比，信标和载体位置
subplot 221,plot(avp(:,end),r2d(avp(:,7)),'M--')
xygo('t/s','lat'),title('lat comparison')
if size(BCN{1},1)==1
    for i=1:length(BCN)
        hold on,plot(avp(:,end),r2d(BCN{i}(1))*ones(size(avp(:,end))));
    end
    legend('trj',leg{:});
    subplot 222,plot(avp(:,end),r2d(avp(:,8)),'M--')
    xygo('t/s','lon'),title('lon comparison')
    for i=1:length(BCN)
        hold on,plot(avp(:,end),r2d(BCN{i}(2))*ones(size(avp(:,end))));
    end
    legend('trj',leg{:});
else
    for i=1:length(BCN)
        hold on,plot(avp(:,end),r2d(BCN{i}(:,1)));
    end
    legend('trj',leg{:});
    subplot 222,plot(avp(:,end),r2d(avp(:,8)),'M--')
    xygo('t/s','lon'),title('lon comparison')
    for i=1:length(BCN)
        hold on,plot(avp(:,end),r2d(BCN{i}(:,2)));
    end
    legend('trj',leg{:});
end
subplot 223,xygo('t/s','R / ( m )'),title('beacon range comparison')
for i=1:length(RNG)
    plot(avp(:,end),RNG{i});hold on;
end
legend(leg);
subplot 224,plot(r2d(avp(:,8)),r2d(avp(:,7)),'--');
hold on;plot(r2d(avp(1,8)),r2d(avp(1,7)),'.'); 
if size(BCN{1},1)>1
    for i=1:length(BCN)
        hold on;plot(r2d(BCN{i}(:,2)),r2d(BCN{i}(:,1)));
    end
else
    for i=1:length(BCN)
        hold on;plot(r2d(BCN{i}(:,2)),r2d(BCN{i}(:,1)),'*');
    end
end
legend('trj','start',leg{:});
legend('Location','northwest');
xygo('lon','lat')
end


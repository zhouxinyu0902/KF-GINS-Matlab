function [starttime,endtime]=week_sec2utc(num,week)
% 使用输入的启动时间及GNSS周内秒，计算整段数据的起始和结束时间
% num(1)：启动时间
% num(2)：周内秒，选择的num为整段数据的末尾
starttime = utc_week_seconds_to_bjt_with_leap(week,num(2)-num(1),0);
endtime = utc_week_seconds_to_bjt_with_leap(week,num(2),0);
end

  function index= conclude_auax(AUAX)
% -------AUAX数据概述---------
unique_values1 = unique(AUAX(end-2,:));
% 为数据创建状态字符串单元格数组
state1 = cell(1, length(unique_values1));
for i = 1:length(unique_values1)
    state1{i} = dec2hex(unique_values1(i));
end
% 动态创建格式字符串
formatStr1 = repmat('%s,', 1, length(unique_values1));
formatStr1 = formatStr1(1:end-1); % 移除最后一个逗号
fprintf('状态字有：');
fprintf([formatStr1 '\n'], state1{:});
[starttime,endtime]=week_sec2utc(AUAX(1:2,end),2386);
fprintf('起始时间: %s\n', starttime);
fprintf('结束时间: %s\n', endtime);

% 分开状态
index.useless=find(AUAX(end-2,:)==hex2dec('0'));
index.coarse=find(AUAX(end-2,:)==hex2dec('5840'));
index.fine=find(AUAX(end-2,:)==hex2dec('5820'));
index.nav=find(AUAX(end-2,:)==hex2dec('4300'));
N = floor((length(index.fine)+10)/36000);
leap_sec = 0;
beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2386, AUAX(2,index.fine(1+(N-1)*6000)), leap_sec);
fprintf('精对准开始时间为: %s \n',beijing_time_precise)

beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2386, AUAX(2,index.fine(end)), leap_sec);
fprintf('精对准结束时间为: %s \n',beijing_time_precise)

beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2386, AUAX(2,index.fine(end)+1), leap_sec);
fprintf('导航开始时间为: %s \n',beijing_time_precise)

beijing_time_precise = utc_week_seconds_to_bjt_with_leap(2386, AUAX(2,index.nav(end)), leap_sec);
fprintf('导航结束时间为: %s \n',beijing_time_precise)
end


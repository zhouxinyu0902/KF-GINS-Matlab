function bj_time = utc_week_seconds_to_bjt_with_leap(week_number, seconds_of_week, leap_seconds)
% 带闰秒修正的转换函数
%   Inputs:
%       week_number      - GPS周数
%       seconds_of_week  - 周内秒
%       leap_seconds     - 当前的累计闰秒数 (e.g., 18 since 2021)
%   Output:
%       bj_time          - 北京时间的datetime对象

    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
    
    % 从GPS时间转到UTC时间：减去闰秒差
    total_seconds = seconds(week_number * 7 * 24 * 60 * 60 + seconds_of_week - leap_seconds);
    
    utc_time = gps_epoch + total_seconds;
    bj_time = utc_time;
    bj_time.TimeZone = 'UTC+8';
    
end
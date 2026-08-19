function [week_number, seconds_of_week] = bjt_to_utc_week_seconds(year, month, day, hour, minute, second, leap_seconds)
% 将北京时间转换为UTC周数和周内秒
%   Inputs: 年,月,日,时,分,秒,闰秒数
%   Outputs: GPS周数, 周内秒

    % 创建北京时间对象
    bj_time = datetime(year, month, day, hour, minute, second, 'TimeZone', 'UTC+8');
    
    % 转换为UTC时间
    utc_time = bj_time;
    utc_time.TimeZone = 'UTC';
    
    % GPS历元
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
    
    % 计算时间差并加上闰秒
    total_seconds = seconds(utc_time - gps_epoch) + leap_seconds;
    
    % 计算周数和周内秒
    seconds_per_week = 7 * 24 * 3600;
    week_number = floor(total_seconds / seconds_per_week);
    seconds_of_week = mod(total_seconds, seconds_per_week);
end
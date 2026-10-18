function row = experiment03_state_to_nav_row(navstate, param, gps_week)
%EXPERIMENT03_STATE_TO_NAV_ROW 将导航状态转换为统一11列输出格式。
%
% [GPS week, time, latitude(deg), longitude(deg), height(m),
%  velocity N/E/D (m/s), roll/pitch/heading (deg)]

if nargin < 3 || isempty(gps_week)
    gps_week = 0;
end

row = [gps_week, navstate.time, ...
    navstate.pos(1) * param.R2D, ...
    navstate.pos(2) * param.R2D, navstate.pos(3), ...
    navstate.vel(:).', navstate.att(:).' * param.R2D];
end

function IMUFRD = imuFUR2FRD(IMUFUR, sample_interval_s)
% 将测量值前上右转为kfgins数据前右上
% 将角速度（°/s）和加速度（m/s^2）转为角度增量和速度增量
% sample_interval_s 可为标量或逐行采样周期。省略时为兼容旧数据，
% 仍使用 0.01 s（100 Hz）。

if nargin < 2 || isempty(sample_interval_s)
    sample_interval_s = 0.01;
end
if isscalar(sample_interval_s)
    sample_interval_s = repmat(sample_interval_s, size(IMUFUR, 1), 1);
else
    sample_interval_s = sample_interval_s(:);
end
if numel(sample_interval_s) ~= size(IMUFUR, 1) || ...
        any(~isfinite(sample_interval_s)) || any(sample_interval_s <= 0)
    error('imuFUR2FRD:InvalidSampleInterval', ...
        'sample_interval_s must be positive and match the IMU row count.');
end

IMUFRD=IMUFUR(:,1:7);
IMUFRD(:,1)=IMUFUR(:,8);
IMUFRD(:,2)=IMUFUR(:,1);
IMUFRD(:,3)=IMUFUR(:,3);
IMUFRD(:,4)=-IMUFUR(:,2);
IMUFRD(:,5)=IMUFUR(:,4);
IMUFRD(:,6)=IMUFUR(:,6);
IMUFRD(:,7)=-IMUFUR(:,5);
IMUFRD(:,2:4)=IMUFRD(:,2:4).*sample_interval_s/180*pi;
IMUFRD(:,5:7)=IMUFRD(:,5:7).*sample_interval_s;
end

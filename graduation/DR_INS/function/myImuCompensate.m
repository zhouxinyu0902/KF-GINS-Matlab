function imu = myImuCompensate(imu, navstate, dt)
%MYIMUCOMPENSATE Compensate IMU increments with the current error estimates.

if dt <= 0
    error('IMU sample interval must be positive.');
end

imu(2:4) = (imu(2:4) - dt * navstate.gyrbias) ./ ...
    (ones(3, 1) + navstate.gyrscale);
imu(5:7) = (imu(5:7) - dt * navstate.accbias) ./ ...
    (ones(3, 1) + navstate.accscale);

end

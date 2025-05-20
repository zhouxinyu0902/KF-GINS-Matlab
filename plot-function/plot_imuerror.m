% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2022.11.30
% -------------------------------------------------------------------------

%% importdata imuerror
% imuerrorfile = 'dataset1/ImuError.txt';
imuerrorfile = 'dataset-simu/ImuError.txt';
err = importdata(imuerrorfile);

figure()
plot(err(:, 1), err(:, 2:4))
title('GyroBias');
xlabel('Time[s]');
ylabel('gb[deg/h]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(err(:, 1), err(:, 5:7))
title('AccelBias');
xlabel('Time[s]');
ylabel('ab[mGal]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(err(:, 1), err(:, 8:10))
title('GyroScale');
xlabel('Time[s]');
ylabel('gs[ppm]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(err(:, 1), err(:, 11:13));
title('AccelScale');
xlabel('Time[s]');
ylabel('as[ppm]');
grid("on");
legend('X', 'Y', 'Z');

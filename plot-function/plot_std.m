% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2022.11.30
% -------------------------------------------------------------------------


%% importdata std
% stdfile = 'dataset1/NavSTD.txt';
stdfile = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据\output/NavSTD.txt';
% stdfile = stdpath;
std = importdata(stdfile);

myfigurestartup(5,5,'prese');
plot(std(:, 1), std(:, 2:4))
title('Position STD');
xlabel('Time[s]');
ylabel('pos[m]');
legend('X', 'Y', 'Z');
grid("on");

myfigurestartup(5,5,'prese');
plot(std(:, 1), std(:, 5:7))
title('Velocity STD');
xlabel('Time[s]');
ylabel('vel[m/s]');
grid("on");
legend('X', 'Y', 'Z');

myfigurestartup(5,5,'prese');
plot(std(:, 1), std(:, 8:10))
title('Attitude STD');
xlabel('Time[s]');
ylabel('att[deg]');
grid("on");
legend('X', 'Y', 'Z');

myfigurestartup(5,5,'prese');
plot(std(:, 1), std(:, 11:13))
title('GyroBias STD');
xlabel('Time[s]');
ylabel('gb[deg/h]');
grid("on");
legend('X', 'Y', 'Z');

myfigurestartup(5,5,'prese');
plot(std(:, 1), std(:, 14:16))
title('AccelBias STD');
xlabel('Time[s]');
ylabel('ab[mGal]');
grid("on");
legend('X', 'Y', 'Z');
% 
% myfigurestartup(5,5,'prese');
% plot(std(:, 1), std(:, 17:19))
% title('GyroScale STD');
% xlabel('Time[s]');
% ylabel('gs[ppm]');
% grid("on");
% legend('X', 'Y', 'Z');

% myfigurestartup(5,5,'prese');
% plot(std(:, 1), std(:, 20:22));
% title('AccelScale STD');
% xlabel('Time[s]');
% ylabel('as[ppm]');
% grid("on");
% legend('X', 'Y', 'Z');

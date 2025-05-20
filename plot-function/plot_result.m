% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2022.11.30
% -------------------------------------------------------------------------
function plot_result(navpath,type)
% importdata navresult
% navpath = "dataset1/NavResult.nav";
% navpath = 'dataset1/pcafile.txt';
% navpath = 'dataset1/pure_ins_PSINS.txt';
% navpath = 'dataset1/pure_ins.txt';
navdata = importdata(navpath);
if nargin<2 
    type="full";
end
if type=="full"
    % velocity
    myfigurestartup(10,10,'prese')
    subplot 323
    plot(navdata(:, 2), navdata(:, 6:8));
    title('Velocity');
    legend('North', 'East', 'Down');
    xlabel('Time[s]');
    ylabel('Vel[m/s]');
    grid("on");

    % attitude
    subplot 321
    plot(navdata(:, 2), navdata(:, 9:10));
    title('Attitude');
    legend('Roll', 'Pitch');
    xlabel('Time[s]');
    ylabel('Att[deg]');
    grid("on");

    subplot 322
    plot(navdata(:, 2), navdata(:, 11));
    title('Attitude');
    legend('Yaw');
    xlabel('Time[s]');
    ylabel('Att[deg]');
    grid("on");

    % position
    param = Param();
    blh = navdata(:, 3:5);
    blh(:, 1) = blh(:, 1) * param.D2R;
    blh(:, 2) = blh(:, 2) * param.D2R;
    first_blh = blh(1, 1:3);

    [rm, rn] = getRmRn(first_blh(1), param);
    h = first_blh(2);
    DR = diag([rm + h, (rn + h)*cos(first_blh(1)), -1]);

    % blh to ned
    pos = zeros(size(blh));
    for i = 1:size(pos, 1)
        delta_blh = blh(i, :) - first_blh;
        delta_pos = DR * delta_blh';
        pos(i, :) = delta_pos';
    end

    %% plane position
    subplot(3,2,[4,6])
    plot(pos(1, 2), pos(1, 1),'*');hold on
    plot(pos(:, 2), pos(:, 1));
    title('Position');
    xlabel('East[m]');
    ylabel('North[m]');
    grid("on");

    %% height
    % subplot 325
    % plot(navdata(:, 2), navdata(:, 5));
    % title('Height');
    % xlabel('Time[s]');
    % ylabel('Height[m]');
    % grid("on");

    subplot 325
    plot(navdata(:, 2), pos(:, 1),navdata(:, 2), pos(:, 2),navdata(:, 2), pos(:, 3));
    title('Height');
    xlabel('Time[s]');
    ylabel('Height[m]');
    grid("on");
elseif type=="single"
    
    myfigurestartup(5,5,'prese')
    plot(navdata(:, 2), navdata(:, 6:8));
    title('Velocity');
    legend('North', 'East', 'Down');
    xlabel('Time[s]');
    ylabel('Vel[m/s]');
    grid("on");

    % attitude
    figure();
    plot(navdata(:, 2), navdata(:, 9:10));
    title('Attitude');
    legend('Roll', 'Pitch');
    xlabel('Time[s]');
    ylabel('Att[deg]');
    grid("on");

    figure();
    plot(navdata(:, 2), navdata(:, 11));
    title('Attitude');
    legend('Yaw');
    xlabel('Time[s]');
    ylabel('Att[deg]');
    grid("on");

    % position
    param = Param();
    blh = navdata(:, 3:5);
    blh(:, 1) = blh(:, 1) * param.D2R;
    blh(:, 2) = blh(:, 2) * param.D2R;
    first_blh = blh(1, 1:3);

    [rm, rn] = getRmRn(first_blh(1), param);
    h = first_blh(2);
    DR = diag([rm + h, (rn + h)*cos(first_blh(1)), -1]);

    % blh to ned
    pos = zeros(size(blh));
    for i = 1:size(pos, 1)
        delta_blh = blh(i, :) - first_blh;
        delta_pos = DR * delta_blh';
        pos(i, :) = delta_pos';
    end

    %% plane position
    figure();
    plot(pos(1, 2), pos(1, 1),'*');hold on
    plot(pos(:, 2), pos(:, 1));
    title('Position');
    xlabel('East[m]');
    ylabel('North[m]');
    grid("on");

    %% height
    figure();
    plot(navdata(:, 2), navdata(:, 5));
    title('Height');
    xlabel('Time[s]');
    ylabel('Height[m]');
    grid("on");
end




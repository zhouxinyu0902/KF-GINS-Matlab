%% 反向推算+滤波,每个测距周期
thisimu2 = imudata(imuindex-1, :)';
lastimu2 = imudata(imuindex, :)';
navstate_2 = navstate;
laststate_2 = navstate;

kf2 = kf;
% kf2.P = kf.P/100;
kf2.P(7:9,7:9) = kf.P(7:9,7:9)*100;

ki2 = ki2+1;

nav = zeros(11, 1);
nav(2, 1) = navstate_2.time;
nav(3:5, 1) = [navstate_2.pos(1) * param.R2D; navstate_2.pos(2) * param.R2D; navstate_2.pos(3)];
nav(6:8, 1) = navstate_2.vel;
nav(9:11, 1) = navstate_2.att * param.R2D;
navdt1s(imuindex-1,:) = nav;


nav112(:,imuindex) = laststate_2.pos;
P_B_store2(:,:,imuindex)=kf2.P(1:2,1:2);
indexrecord2(ki2) = imuindex;
for ii = indexrecord2(ki2)-1:-1:indexrecord2(ki2-1)+1
    if ii==indexrecord2(ki2-1)+1
        laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
        laststate_2.pos(3) = height(ii-1,2);
        % 反向滤波
        if rangeindex == 2
            Rangedata = zeros(1,6);
            Rangedata(4:6) = rangedata3(1,4:6);
            Rangedata(3) = caldot2dot(Rangedata(4:6),pos0');
        else
            Rangedata = rangedata(rangeindex-2,:);
        end
        depthdata = height(ii-1,1:2);
        kf2 = myRangeUpdate_adap(laststate_2, Rangedata, depthdata, kf2);
        [kf2, laststate_2] = myErrorFeedback_range(kf2, laststate_2);

        % if rangeindex>2 & abs(kf2.Z(1))-abs(z(rangeindex-2,1))>40
        %     break;
        % else
        %     nav11(:,ii-1) = laststate_1.pos;
        % end

        z_back(ki2-1,1) = kf2.Z(1);% 残差记录
        z_back(ki2-1,2) = kf2.Znew;
        z_back(ki2-1,3) = kf2.alpha;
        z_back(ki2-1,4) = kf2.d_squared ;
        z_back(ki2-1,5) = kf2.chi2_threshold ;
        z_back(ki2-1,6) = kf2.is_anomaly ;
    else
        laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
        laststate_2.pos(3) = height(ii-1,2);
        kf2 = myInsPropagate_15state(laststate_2, thisimu2, 0.01, kf2);
    end
    lastimu2 = thisimu2;
    thisimu2 = imudata(ii-1, :)';
    navstate_2 = laststate_2;
    % nav112(:,ii) = laststate_2.pos;
    P_B_store2(:,:,ii-1) = kf2.P(1:2,1:2);
    nav = zeros(11, 1);
    nav(2, 1) = navstate_2.time;
    nav(3:5, 1) = [navstate_2.pos(1) * param.R2D; navstate_2.pos(2) * param.R2D; navstate_2.pos(3)];
    nav(6:8, 1) = navstate_2.vel;
    nav(9:11, 1) = navstate_2.att * param.R2D;
    navdt1s(ii-1,:) = nav;
end

fprintf(navdt1sfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navdt1s(indexrecord2(ki2-1):indexrecord2(ki2)-1,:)');

newEndPoint = [d2r(navdt1s(indexrecord2(ki2-1),3:4)),navdt1s(indexrecord2(ki2-1),5)]';
trajectory = flip([d2r(navdt1s(indexrecord2(ki2-1)+1:indexrecord2(ki2)-1,3:4)),...
    navdt1s(indexrecord2(ki2-1)+1:indexrecord2(ki2)-1,5)]',2);
[rotatedTrajectory, scaleFactor, rotationAngle_deg] = rotateAndScaleTrajectory(trajectory, newEndPoint);
rotatedTrajectory = flip(rotatedTrajectory, 2);
rotatedTrajectory = [newEndPoint,rotatedTrajectory];
rotatedTrajectory(1:2,:) = r2d(rotatedTrajectory(1:2,:));
navdt1srotate = navdt1s(indexrecord2(ki2-1):indexrecord2(ki2)-1,:);
navdt1srotate(:,3:4) = rotatedTrajectory(1:2,:)';

NAV(indexrecord2(ki2-1):indexrecord2(ki2)-1,:)=navdt1srotate;
fprintf(navdt1srotatefp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navdt1srotate');
% if rangeindex == 499
%     myfigurestartup(5,5,'prese')
%     plot(NAV(:,4),NAV(:,3))
%     hold on
%     plot(navdt1s(:,4),navdt1s(:,3))
%     legend('前向滤波+旋转','前向滤波')
%     keyboard;
% end
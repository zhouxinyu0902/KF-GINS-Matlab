clear
glvs
avp0 = [[0;0;d2r(0)]; [0;0;0]; [d2r([36;126]);0]];
ts=0.01;
n=3;
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.2); % 加速
seg = trjsegment(seg, 'uniform',      22*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',     42*n);
repeats=1;
trj2= trjsimu(avp0, seg.wat, ts, repeats);


myfigurestartup(7,7,'prese')
insplot(trj2.avp)
fprintf('轨迹总时长%d（s）\n',sum(trj2.wat(:,1)))
pva = avpENU2NED(trj2.avp);
%%
beacon_xyz = [-2000,4000,0];
beacon_rrm = dxyz2pos(beacon_xyz,avp0(7:9));

trj_xyz = pos2dxyz(trj2.avp(1:100:end,7:9),avp0(7:9));

heading = pva(1:100:end,end);
for i=1:length(trj_xyz)
    % 计算方位角
    azimuth(i) = pos2azimuth(trj_xyz(i,1:2),beacon_xyz(1:2),heading(i))+randn*0.1;
    range(i) = norm(trj_xyz(i,1:2)-beacon_xyz(1:2))+randn*2;
    my_pos(1:2,i) = calc_position_from_beacon(beacon_xyz(1:2), range(i), azimuth(i), heading(i)+d2r(randn*0.01));
end
pos_cal = [my_pos',trj_xyz(:,3)];

err = pos_cal-trj_xyz;
radial_err = sqrt(err(:,1).^2+err(:,2).^2);
myfigurestartup(7,7,'prese')
plot(trj_xyz(:,1),trj_xyz(:,2))
hold on
plot(beacon_xyz(1),beacon_xyz(2),'*')
plot(pos_cal(:,1),pos_cal(:,2))
myfigurestartup(7,7,'prese')
subplot 121
plot(trj2.avp(1:100:end,end),err(:,1))
subplot 122
plot(trj2.avp(1:100:end,end),err(:,2))
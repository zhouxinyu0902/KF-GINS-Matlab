glvs
% trj = trjfile('trj10ms_3.mat');
ts = 0.01;  
avp0 = [[0;0;d2r(-90)]; [0;0;0]; d2r([30;120;0])]; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20);
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.20576); 
seg = trjsegment(seg, 'uniform',      3600); 
seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.20576); 
trj = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
%% error setting
imuerr = imuerrset(0.01, 100, 0.001, 10);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([0.5;0.5;5], 0.1, [10;10;10]);
avp00 = avpadderr(trj.avp0, davp0); 
trj = bhsimu(trj, 1, 10, 3, trj.ts); 
%% pure inertial navigation & error plot
% avp = inspure(imu, avp00, trj.bh, 1);
avp = inspure(trj.imu, trj.avp0, 'f', 1);
avp1 = inspure(imu, trj.avp0, 'f', 1);

%%
ins1 = myins('initial',0.01, trj.avp0);
ll = length(trj.avp);
avp_pureins=prealloc(ll,10);
for i=1:ll
    t = trj.imu(i,end);
    ins1 = myins('update',ins1,trj.imu(i,1:6));
    avp_pureins(i,:)=[ins1.avp',t];
end
%%
% avperr = avpcmpplot(trj.avp, avp);
figure
insplot(trj.avp(:,7:9))
hold on
insplot(avp(:,7:9))
hold on
insplot(avp1(:,7:9))
hold on
insplot(avp_pureins(:,7:9))
figure
trjsee(trj.avp,'2d',avp,avp1)
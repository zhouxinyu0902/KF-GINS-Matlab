glvs
load('Leador-A15\data_Leador-A15.mat')



att00=d2r([-2, 5, 120]');
att0=att00;
vel0=[0.0, 0.0, 0.0]';
pos0=[30, 120, 20]';
pos0(1:2)=d2r(pos0(1:2));
avp0=[att0;vel0;pos0];
ts=0.005;
imu(300/0.005,1:6)
ins = myins('initial',ts,avp0);
ins1 = myins('update',ins,imu(300/0.005,1:6));
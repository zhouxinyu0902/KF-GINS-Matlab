close all
clear
glvs
avp0 = [[0;0;d2r(180)]; [0;0;0]; glv.pos0]; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      10); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.5); % 加速
seg = trjsegment(seg, 'uniform',      4370); % 保持原来的状态不变
seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.5); 
seg = trjsegment(seg, 'uniform',      500);
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.5); 
seg = trjsegment(seg, 'turnleft', 90, 1);
seg = trjsegment(seg, 'uniform',      1300);
seg = trjsegment(seg, 'turnleft', 90, 1);
seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.5); 
seg = trjsegment(seg, 'uniform',      500);
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.5); 
seg = trjsegment(seg, 'uniform',      1790);
trj_001= trjsimu(avp0, seg.wat, 0.01, 1); 
trj_005=trjsimu(avp0, seg.wat, 0.05, 1); 
trj_01=trjsimu(avp0, seg.wat, 0.1, 1); 
trj_05=trjsimu(avp0, seg.wat, 0.5, 1); 

insplot(trj_05.avp);
%%
save data_1\avp_5m.mat trj_001 trj_005 trj_01 trj_05

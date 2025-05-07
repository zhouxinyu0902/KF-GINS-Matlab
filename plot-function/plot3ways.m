clear all
data1GINSC=importdata("dataset1\pcafile.txt");
data2GINSM=importdata("dataset1\pure_ins.txt");
data3PSINS=importdata("dataset1\pure_ins_PSINS.txt");
ref=importdata("dataset1\truth.nav");
%% 
ref = ref(ref(:,2) >= 456300, :);
ref = ref(ref(:,2) <= 456900, :);
%% 绘图

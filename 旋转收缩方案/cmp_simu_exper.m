path1='旋转收缩方案/output/pureins.nav';
path11='旋转收缩方案/output_simu/pureins.nav';

truth='旋转收缩方案/input/truth.nav';
truth1='旋转收缩方案/input_simu/truth.txt';

path2='旋转收缩方案/output/RANGE1-adap.nav';
path22='旋转收缩方案/output_simu/RANGE1-adap.nav';

path3='旋转收缩方案/output/rotate+backforward.txt';
path33='旋转收缩方案/output_simu/rotate+backforward.txt';
%%
myfigurestartup(12,5,'prese')
subplot 122
plot_trj(truth,path1,path3)
title('实测数据纯惯导/距离辅助改进算法对比')
subplot 121
plot_trj(truth1,path11,path33)
title('仿真数据纯惯导/距离辅助改进算法对比')
%%
myfigurestartup(10,10,'prese')
subplot 212
calc_radial_error(truth,path1,path2,path3)s
title('实测数据纯惯导/距离辅助改进算法对比')
subplot 211
calc_radial_error(truth1,path11,path22,path33)
title('仿真数据纯惯导/距离辅助改进算法对比')
%%
calc_error(path1,truth)
calc_error(path11,truth1)
%%

calc_error(path2,truth)
calc_error(path22,truth1)
%%
plot_result(truth)

plot_result(truth1)
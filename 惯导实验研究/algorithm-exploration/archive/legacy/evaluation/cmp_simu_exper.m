topic_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
project_root = fileparts(fileparts(topic_dir));
data_root = fullfile(project_root, 'data', 'inertial-experiment');
experiment_output = fullfile(data_root, 'algorithm-exploration', 'navigation-results', 'experiment');
simulation_input = fullfile(data_root, 'algorithm-exploration', 'input', 'simulation', 'case-00');
simulation_output = fullfile(data_root, 'algorithm-exploration', 'navigation-results', 'simulation', 'case-00');

path1=fullfile(experiment_output, 'PureIns.nav');
path11=fullfile(simulation_output, 'pureins.nav');

truth=fullfile(data_root, 'algorithm-exploration', 'input', 'experiment-reference', 'truth.nav');
truth1=fullfile(simulation_input, 'truth.txt');

path2=fullfile(experiment_output, 'AEKF.nav');
path22=fullfile(simulation_output, 'RANGE1-adap.nav');

path3=fullfile(experiment_output, 'GTS-BRC-AEKF.txt');
path33=fullfile(simulation_output, 'rotate+backforward.txt');
%%
myfigurestartup(12,5,'prese');
subplot 122
plot_trj(truth,path1,path3)
title('实测数据纯惯导/距离辅助改进算法对比')
subplot 121
plot_trj(truth1,path11,path33)
title('仿真数据纯惯导/距离辅助改进算法对比')
%%
myfigurestartup(10,10,'prese');
subplot 212
calc_radial_error(truth,path1,path2,path3)
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

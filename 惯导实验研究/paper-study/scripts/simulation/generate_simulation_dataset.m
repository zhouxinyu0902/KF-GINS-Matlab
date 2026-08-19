%% PSINS轨迹与IMU数据仿真
clear;
clc;
paper_paths = setup_paper_study();
glvs;
%% 1. 仿真参数
dataset_id = 2;     % 可选 1 或 2
ts = 0.01;          % IMU采样周期，单位：s
gps_week = 2300;     % GPS周，请根据实验设置修改
start_sow = 0;       % 起始周内秒，单位：s

switch dataset_id
    case 1
        output_path = paper_paths.simulation_dataset1;
    case 2
        output_path = paper_paths.simulation_dataset2;
    otherwise
        error('不支持的仿真数据集编号：%d', dataset_id);
end
if ~isfolder(output_path)
    mkdir(output_path);
end
figure_dir = fullfile(paper_paths.simulation_figures_tables, ...
    sprintf('dataset%d', dataset_id), 'data-generation');
if ~isfolder(figure_dir)
    mkdir(figure_dir);
end

%% 2. 初始状态
% PSINS AVP排列：[pitch; roll; yaw; vE; vN; vU; lat; lon; height]
avp0 = [0; 0; d2r(135); ...
        0; 0; 0; ...
        glv.pos0];

%% 3. 生成运动轨迹
seg = trjsegment([], 'init', 0);
seg = trjsegment(seg, 'accelerate', 10, [], 0.20577);
seg = trjsegment(seg, 'uniform', 5000);

trj = trjsimu(avp0, seg.wat, ts, 1);

fprintf("轨迹总时长：%.2f s\n", sum(trj.wat(:, 1)));

% 绘制轨迹
insplot(trj.avp);
trajectory_figure = gcf;
exportgraphics(trajectory_figure, ...
    fullfile(figure_dir, 'generated-trajectory.png'), 'Resolution', 300);
savefig(trajectory_figure, ...
    fullfile(figure_dir, 'generated-trajectory.fig'));

%% 4. 添加IMU误差
gyro_bias = 0.01;
acc_bias = 7;
gyro_noise = 0.0005;
acc_noise = 10e-6 * 1e5 / 3600;

imu_error = imuerrset(gyro_bias, acc_bias, gyro_noise, acc_noise);
imu_with_error = imuadderr(trj.imu, imu_error);

%% 5. 坐标系转换
% AVP：ENU转换为NED
reference_pva = avpENU2NED(trj.avp);
reference_pva(:, 1) = gps_week;
reference_pva(:, 2) = reference_pva(:, 2) + start_sow;

% IMU：RFU转换为FRD
imu_simulation = imuRFU2FRD(imu_with_error);
imu_simulation(:, 1) = imu_simulation(:, 1) + start_sow;

%% 6. 写入truth.nav
% KF-GINS真值文件格式：
% 1    GPS周
% 2    周内秒，s
% 3~5  纬度、经度、高程，deg、deg、m
% 6~8  北、东、地速度，m/s
% 9~11 横滚角、俯仰角、航向角，deg

truth_file = fullfile(output_path, "truth.nav");
truth_fid = fopen(truth_file, "w");

if truth_fid == -1
    error("无法创建真值文件：%s", truth_file);
end

try
    fprintf(truth_fid, ...
        "%d %.4f %.10f %.10f %.4f " + ...
        "%.8f %.8f %.8f %.8f %.8f %.8f\n", ...
        reference_pva');

    fclose(truth_fid);
    fprintf("真值文件已写入：%s\n", truth_file);
catch ME
    fclose(truth_fid);
    error("写入truth.nav失败：%s", ME.message);
end

%% 7. 写入IMU_120.txt
% KF-GINS IMU文件格式：
% 1    周内秒，s
% 2~4  X、Y、Z轴角增量，rad
% 5~7  X、Y、Z轴速度增量，m/s

imu_file = fullfile(output_path, "IMU_120.txt");
imu_fid = fopen(imu_file, "w");
if imu_fid == -1
    error("无法创建IMU文件：%s", imu_file);
end

try
    fprintf(imu_fid, ...
        "%.4f %.12e %.12e %.12e %.12e %.12e %.12e\n", ...
        imu_simulation');

    fclose(imu_fid);
    fprintf("IMU文件已写入：%s\n", imu_file);
catch ME
    fclose(imu_fid);
    error("写入IMU_120.txt失败：%s", ME.message);
end

%% 8. 生成测距数据
rangedataget(output_path, 'right', 0, 1, 0, 0);

fprintf("全部仿真数据生成完成：%s\n", output_path);
fprintf("轨迹图已保存：%s\n", figure_dir);


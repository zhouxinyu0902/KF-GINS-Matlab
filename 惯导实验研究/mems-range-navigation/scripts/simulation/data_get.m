% 主脚本
% │
% ├─ 1. 环境初始化
% ├─ 2. 轨迹生成
% ├─ 3. IMU误差仿真
% ├─ 4. IMU / Truth 保存
% ├─ 5. GNSS仿真
% ├─ 6. 深度计仿真
% └─ 7. 声学测距仿真
%       ├─ generate_static_beacon_range()
%       ├─ save_static_beacon_range()
%       └─ plot_trajectory_and_beacons_m()


clear
clc
%% 1. 环境初始化
% =========================================================================
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_mems_range_navigation();
glvs
%%  2. 初始状态
% =========================================================================
% avp0:
% [attitude;
%  velocity;
%  position]
%
% 姿态：rad
% 速度：m/s
% 位置：[lat, lon, h]，lat/lon单位rad
avp0 = [
    [0; 0; d2r(0)];
    [0; 0; 0];
    [d2r([28.227074; 112.996809]); 0]
];
ts = 0.01;
%%  3. 构造仿真轨迹
% =========================================================================

% -------------------------------------------------------------------------
% 原始轨迹方案
% -------------------------------------------------------------------------
%
% xxx = [];
% seg = trjsegment(xxx, 'init',       0);
% seg = trjsegment(seg, 'uniform',    20);
%
% seg = trjsegment( ...
%     seg, ...
%     'accelerate', ...
%     3/(20/9.8), ...
%     xxx, ...
%     20/9.8);
%
% for i = 1:7
%
%     seg = trjsegment(seg, 'uniform',   22);
%     seg = trjsegment(seg, 'turnleft',  18, 5);
%
%     seg = trjsegment(seg, 'uniform',   64);
%     seg = trjsegment(seg, 'turnleft',  18, 5);
%
%     seg = trjsegment(seg, 'uniform',   64);
%     seg = trjsegment(seg, 'turnleft',  18, 5);
%
%     seg = trjsegment(seg, 'uniform',   64);
%     seg = trjsegment(seg, 'turnleft',  18, 5);
%
%     seg = trjsegment(seg, 'uniform',   42);
%
% end
%
% seg = trjsegment(seg, 'uniform',      22);
% seg = trjsegment(seg, 'turnleft',     18, 5);
%
% seg = trjsegment(seg, 'uniform',      64);
% seg = trjsegment(seg, 'turnleft',     18, 5);
%
% seg = trjsegment(seg, 'uniform',      50);
%
% seg = trjsegment( ...
%     seg, ...
%     'deaccelerate', ...
%     3/(20/9.8), ...
%     xxx, ...
%     20/9.8);

% -------------------------------------------------------------------------
% 当前使用轨迹
% -------------------------------------------------------------------------
n = 3;
xxx = [];
seg = trjsegment(xxx, 'init', 0);
% 初始静止
seg = trjsegment(seg, 'uniform', 20);
% 加速
seg = trjsegment(seg, 'accelerate', 10, xxx, 0.2);
% -------------------------------------------------------------------------
% 第1圈
% -------------------------------------------------------------------------
% seg = trjsegment(seg, 'uniform', 22*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 42*n);
% 
% 
% % -------------------------------------------------------------------------
% % 第2圈
% % -------------------------------------------------------------------------
% seg = trjsegment(seg, 'uniform', 22*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 42*n);
% 
% 
% % -------------------------------------------------------------------------
% % 第3圈
% % -------------------------------------------------------------------------
% seg = trjsegment(seg, 'uniform', 22*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 64*n);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% 
% seg = trjsegment(seg, 'uniform', 42*n);
% 
% 
% % -------------------------------------------------------------------------
% % 第4圈
% % -------------------------------------------------------------------------
for i = 1:4
    seg = trjsegment(seg, 'uniform', 22*n);
    seg = trjsegment(seg, 'turnleft', 18, 5);

    seg = trjsegment(seg, 'uniform', 64*n);
    seg = trjsegment(seg, 'turnleft', 18, 5);

    seg = trjsegment(seg, 'uniform', 64*n);
    seg = trjsegment(seg, 'turnleft', 18, 5);

    seg = trjsegment(seg, 'uniform', 64*n);
    seg = trjsegment(seg, 'turnleft', 18, 5);

    seg = trjsegment(seg, 'uniform', 42*n);
end

%%  4. 生成理想轨迹及理想IMU
% =========================================================================
repeats = 1;
trj2 = trjsimu( ...
    avp0, ...
    seg.wat, ...
    ts, ...
    repeats);
% 绘图
myfigurestartup(7, 7, 'prese');
insplot(trj2.avp);
fprintf( ...
    '轨迹总时长：%.2f s\n', ...
    sum(trj2.wat(:,1)));


%%   5. IMU误差模型
% =========================================================================

% -------------------------------------------------------------------------
% 原参数
% -------------------------------------------------------------------------
%
% eb  = 0.003;
% db  = 7;
% web = 0.0003;
%
% wdb = 1e-6 * 1e5 / 3600;
% -------------------------------------------------------------------------
% 当前参数
% -------------------------------------------------------------------------
eb = 0.1;
db = [300, ...
     300, ...
    -300
];
web = 0.01;
wdb = 30;

% -------------------------------------------------------------------------
% 固定随机种子，保证每次仿真结果一致
% -------------------------------------------------------------------------
rng(1);

imuerr = imuerrset( ...
    eb, ...
    db, ...
    web, ...
    wdb, ...
    web, ...
    4, ...
    wdb, ...
    4, ...
    5, ...
    10, ...
    5, ...
    10, ...
    10, ...
    10, ...
    10);


% 也可使用基础版本：
%
% imuerr = imuerrset( ...
%     eb, ...
%     db, ...
%     web, ...
%     wdb);


% 给理想IMU添加误差
trjimu_line = imuadderr( ...
    trj2.imu, ...
    imuerr);
%% 6. 保存 IMU 和 Truth 数据
% =========================================================================

input_path = fullfile( ...
    paths.simulation, ...
    'input');

if ~exist(input_path, 'dir')
    mkdir(input_path);
end

%% IMU：RFU -> FRD
% -------------------------------------------------------------------------

imu_line = imuRFU2FRD(trjimu_line);

imu_path = fullfile( ...
    input_path, ...
    'imu.nav');

imu_fp = fopen(imu_path, 'wt');

if imu_fp < 0
    error('无法创建IMU文件：%s', imu_path);
end

fprintf( ...
    imu_fp, ...
    '%.9f %.10f %.10f %.10f %.10f %.10f %.10f\n', ...
    imu_line');

fclose(imu_fp);

fprintf( ...
    'IMU数据已写入：%s\n', ...
    imu_path);


%%  Truth：ENU -> NED
% -------------------------------------------------------------------------

pva_ref_line = avpENU2NED(trj2.avp);

truth_path = fullfile( ...
    input_path, ...
    'truth.nav');

truth_fp = fopen(truth_path, 'wt');

if truth_fp < 0
    error('无法创建Truth文件：%s', truth_path);
end

fprintf( ...
    truth_fp, ...
    '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n', ...
    pva_ref_line');

fclose(truth_fp);

fprintf( ...
    'Truth数据已写入：%s\n', ...
    truth_path);


%%   7. 读取参考结果
% =========================================================================

clc

glvs

truth = importdata(truth_path);

truth_dt = truth(2,2) - truth(1,2);

fprintf( ...
    '参考结果的频率：%.2f Hz\n', ...
    1 / truth_dt);


output_path = input_path;


%%   8. 仿真 GNSS 数据
% =========================================================================

% GNSS：
% 100 Hz truth -> 1 Hz GNSS

Re = 6378137;

gnss = truth(100:100:end, 2:5);


% GNSS位置噪声
% 水平位置标准差：0.02 m
% 高程标准差：0.02 m

gnss(:,2:3) = gnss(:,2:3) + ...
    normrnd( ...
        0, ...
        0.02 / Re * 180 / pi, ...
        size(gnss(:,2:3)));

gnss(:,4) = gnss(:,4) + ...
    normrnd( ...
        0, ...
        0.02, ...
        size(gnss(:,4)));


% 保存GNSS

gnss_output_file = fullfile( ...
    output_path, ...
    'gnss.txt');

try

    writematrix( ...
        gnss, ...
        gnss_output_file, ...
        'Delimiter', ...
        ' ');

    fprintf( ...
        'GNSS信息已成功写入：%s\n', ...
        gnss_output_file);

catch ME

    error( ...
        'GNSS文件写入失败：%s', ...
        ME.message);

end


%%   9. 高度 / 深度信息仿真
% =========================================================================

% -------------------------------------------------------------------------
% 一阶Markov高度误差示例
% -------------------------------------------------------------------------
%
% BIAS  = 0;
% SIGMA = 0.5;
% TAU   = 10;
% DT    = 0.01;
%
% h_true_vector = truth(:,5);
%
% [t, h_meas, h_err] = generate_baro_alt_sim( ...
%     h_true_vector, ...
%     BIAS, ...
%     SIGMA, ...
%     TAU, ...
%     DT);
%
%
% figure;
%
% subplot(2,1,1);
%
% plot(t, h_err);
%
% title( ...
%     ['一阶马尔科夫过程误差 (\tau = ', ...
%      num2str(TAU), ...
%      's, \sigma = ', ...
%      num2str(SIGMA), ...
%      'm)']);
%
% xlabel('时间 (s)');
% ylabel('误差 (m)');
% grid on;
%
%
% subplot(2,1,2);
%
% plot( ...
%     t, ...
%     h_true_vector, ...
%     'k-', ...
%     'LineWidth', ...
%     1.5);
%
% hold on;
%
% plot( ...
%     t, ...
%     h_meas, ...
%     'r:', ...
%     'LineWidth', ...
%     1);
%
% title('气压高度计仿真测量值');
%
% xlabel('时间 (s)');
% ylabel('高度 (m)');
%
% legend( ...
%     '真实高度', ...
%     '测量高度', ...
%     'Location', ...
%     'best');
%
% grid on;
%
%
% height_markov = [
%     truth(:,2), ...
%     h_meas
% ];
%
%
% height_markov_output_file = fullfile( ...
%     output_path, ...
%     'height-markov-10Hz.txt');
%
%
% try
%
%     writematrix( ...
%         height_markov, ...
%         height_markov_output_file, ...
%         'Delimiter', ...
%         ' ');
%
%     fprintf( ...
%         '高度信息已成功写入：%s\n', ...
%         height_markov_output_file);
%
% catch ME
%
%     error( ...
%         '高度文件写入失败：%s', ...
%         ME.message);
%
% end


% -------------------------------------------------------------------------
% 当前使用：100 Hz 高度数据
% -------------------------------------------------------------------------

height = truth(:, [2, 5]);


% -------------------------------------------------------------------------
% 可添加深度计误差
%
% 深度计误差示例：
% 0.02% × 深度
%
% 1500 m时：
% 0.0002 × 1500 = 0.3 m
% -------------------------------------------------------------------------
%
% height(:,2) = height(:,2) + ...
%     normrnd(0, 0.5, size(height(:,2)));


height_output_file = fullfile( ...
    output_path, ...
    'height-100Hz.txt');


try

    writematrix( ...
        height, ...
        height_output_file, ...
        'Delimiter', ...
        ' ');

    fprintf( ...
        '高度信息已成功写入：%s\n', ...
        height_output_file);

catch ME

    error( ...
        '高度文件写入失败：%s', ...
        ME.message);

end


%%   10. 仿真静态信标测距数据
% =========================================================================

% -------------------------------------------------------------------------
% 局部坐标系参考原点
%
% beacon_origin：
% [lat, lon, h]
%
% lat/lon：rad
% h：m
%
% 此处以轨迹初始经纬度、海平面高度0作为局部坐标原点
% -------------------------------------------------------------------------

beacon_origin = [
    d2r(truth(1,3)), ...
    d2r(truth(1,4)), ...
    0
]';


% 信标相对局部原点的位置
%
% 坐标：
% [East, North, Up]
%
% 单位：m
% -------------------------------------------------------------------------

dxyz_original = [
    200, 400, 0
];


% -------------------------------------------------------------------------
% 多信标示例
% -------------------------------------------------------------------------
%
% dxyz_original = [
%       0,  200, 0;
%    -250,  200, 0;
%    -400,  100, 0;
%    -400, -150, 0;
%    -250, -250, 0;
%       0, -250, 0;
%     100, -150, 0;
%     100,  100, 0;
%    -200,    0, 0
% ];

% 信标整体旋转
% -------------------------------------------------------------------------
beacon_rotation_deg = 0;

%%  声学距离测量噪声
%
% 单位：m
%
% 0 = 理想测距
% 5 = 5 m标准差
% -------------------------------------------------------------------------

range_sigma = 0;


%%  构造静态信标测距
% -------------------------------------------------------------------------

[range_beacon, beacon_info] = ...
    generate_static_beacon_range( ...
        truth, ...
        beacon_origin, ...
        dxyz_original, ...
        beacon_rotation_deg, ...
        range_sigma);


%%  绘制轨迹和信标
% -------------------------------------------------------------------------

plot_trajectory_and_beacons_m( ...
    beacon_info.trajectory_xyz, ...
    beacon_info.beacon_xyz);


%% 保存测距数据
% -------------------------------------------------------------------------

save_static_beacon_range( ...
    range_beacon, ...
    output_path, ...
    'range_static');


%%  输出信标信息
% -------------------------------------------------------------------------

fprintf('\n');

fprintf( ...
    '静态信标数量：%d\n', ...
    beacon_info.beacon_num);

fprintf( ...
    '信标整体旋转角：%.2f deg\n', ...
    beacon_rotation_deg);

fprintf( ...
    '测距噪声标准差：%.2f m\n', ...
    range_sigma);


for i = 1:beacon_info.beacon_num

    fprintf( ...
        ['Beacon %d：' ...
         'ENU = [%.2f, %.2f, %.2f] m，' ...
         'LLH = [%.8f deg, %.8f deg, %.2f m]\n'], ...
        i, ...
        beacon_info.beacon_xyz(i,1), ...
        beacon_info.beacon_xyz(i,2), ...
        beacon_info.beacon_xyz(i,3), ...
        beacon_info.beacon_deg(i,1), ...
        beacon_info.beacon_deg(i,2), ...
        beacon_info.beacon_deg(i,3));

end


%%   11. 移动信标仿真
%
%  当前暂不启用
% =========================================================================

% glvs
%
% pos0 = d2r(truth(1,3:5));
%
% dxyz = [
%     200, ...
%     200, ...
%     0
% ];
%
%
% avp0_beacon = [
%     [0; 0; d2r(91)];
%     [0; 0; 0];
%     dxyz2pos(dxyz, pos0')'
% ];
%
%
% ts_beacon = 1;
%
%
% xxx = [];
%
%
% seg_beacon = trjsegment( ...
%     xxx, ...
%     'init', ...
%     0);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'accelerate', ...
%     5, ...
%     xxx, ...
%     0.1);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'uniform', ...
%     1000);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'turnleft', ...
%     150, ...
%     0.6);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'uniform', ...
%     500);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'turnright', ...
%     20, ...
%     3);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'uniform', ...
%     500);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'turnleft', ...
%     70, ...
%     0.6);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'uniform', ...
%     600);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'turnright', ...
%     18, ...
%     4);
%
%
% seg_beacon = trjsegment( ...
%     seg_beacon, ...
%     'uniform', ...
%     400);
%
%
% trj_beacon = trjsimu( ...
%     avp0_beacon, ...
%     seg_beacon.wat, ...
%     ts_beacon, ...
%     1);


%% =========================================================================
%  局部函数
% =========================================================================


function [range_beacon, info] = generate_static_beacon_range( ...
    truth, ...
    beacon_origin, ...
    dxyz_original, ...
    rotation_deg, ...
    range_sigma)
%GENERATE_STATIC_BEACON_RANGE
%
% 构造静态信标声学距离仿真数据。
%
%
% 输入：
% -------------------------------------------------------------------------
% truth
%   参考导航结果
%
%   至少包含：
%
%   truth(:,2)   时间 [s]
%   truth(:,3)   纬度 [deg]
%   truth(:,4)   经度 [deg]
%   truth(:,5)   高度 [m]
%
%
% beacon_origin
%   局部ENU坐标系参考原点：
%
%   [latitude, longitude, height]
%
%   纬经度：rad
%   高度：m
%
%
% dxyz_original
%   信标在局部ENU坐标系下的位置：
%
%   N × 3
%
%   [East, North, Up]
%
%   单位：m
%
%
% rotation_deg
%   信标布局整体绕Z轴旋转角
%
%   单位：deg
%
%
% range_sigma
%   声学斜距测量噪声标准差
%
%   单位：m
%
%   0 表示无噪声
%
%
% 输出：
% -------------------------------------------------------------------------
% range_beacon
%
%   1 × N cell
%
%   每一个cell：
%
%   [
%       time,
%       slant_range,
%       horizontal_range,
%       beacon_lat,
%       beacon_lon,
%       beacon_height
%   ]
%
%
% info
%
%   info.beacon_num
%   info.beacon_xyz
%   info.beacon_pos
%   info.beacon_deg
%   info.trajectory_xyz
%   info.horizontal_distances
%   info.slant_distances


    %% --------------------------------------------------------------------
    % 输入检查
    % ---------------------------------------------------------------------

    if size(truth, 2) < 5
        error( ...
            ['truth至少需要包含5列：', ...
             'time / latitude / longitude / height']);
    end


    if size(beacon_origin, 1) ~= 3
        error( ...
            'beacon_origin必须为3×1向量：[lat, lon, h]');
    end


    if size(dxyz_original, 2) ~= 3
        error( ...
            'dxyz_original必须为N×3矩阵：[East, North, Up]');
    end


    if range_sigma < 0
        error( ...
            'range_sigma不能小于0');
    end


    beacon_num = size(dxyz_original, 1);

    sample_num = size(truth, 1);


    %% --------------------------------------------------------------------
    % 信标布局整体旋转
    % ---------------------------------------------------------------------

    theta = deg2rad(rotation_deg);


    Rz = [
        cos(theta), -sin(theta), 0;
        sin(theta),  cos(theta), 0;
        0,           0,          1
    ];


    beacon_xyz = ...
        (Rz * dxyz_original')';


    %% --------------------------------------------------------------------
    % 信标 ENU -> 经纬高
    % ---------------------------------------------------------------------

    beacon_pos = dxyz2pos( ...
        beacon_xyz, ...
        beacon_origin);


    % beacon_pos：
    %
    % [lat(rad), lon(rad), height(m)]


    beacon_deg = beacon_pos;

    beacon_deg(:,1:2) = ...
        r2d(beacon_deg(:,1:2));


    %% --------------------------------------------------------------------
    % Truth 经纬高 -> 局部ENU
    % ---------------------------------------------------------------------

    trajectory_pos = truth(:,3:5);

    trajectory_pos(:,1:2) = ...
        d2r(trajectory_pos(:,1:2));


    trajectory_xyz = pos2dxyz( ...
        trajectory_pos, ...
        beacon_origin);


    %% --------------------------------------------------------------------
    % 计算水平距离和三维斜距
    % ---------------------------------------------------------------------

    horizontal_distances = ...
        zeros(sample_num, beacon_num);


    slant_distances = ...
        zeros(sample_num, beacon_num);


    for i = 1:beacon_num

        delta_xyz = ...
            trajectory_xyz - beacon_xyz(i,:);


        % 二维水平距离
        horizontal_distances(:,i) = hypot( ...
            delta_xyz(:,1), ...
            delta_xyz(:,2));


        % 三维斜距
        slant_distances(:,i) = sqrt( ...
            delta_xyz(:,1).^2 + ...
            delta_xyz(:,2).^2 + ...
            delta_xyz(:,3).^2);

    end


    %% --------------------------------------------------------------------
    % 构造测距数据
    % ---------------------------------------------------------------------
    range_beacon = ...
        cell(1, beacon_num);
    for i = 1:beacon_num

        horizontal_range = ...
            horizontal_distances(:,i);
        slant_range = ...
            slant_distances(:,i);

        % ---------------------------------------------------------------
        % 声学测量噪声
        % ---------------------------------------------------------------
        if range_sigma > 0

            range_noise = ...
                range_sigma * randn(sample_num,1);

            slant_range = ...
                slant_range + range_noise;
            horizontal_range = ...
                horizontal_range + range_noise;
        end
        % ---------------------------------------------------------------
        % 将信标位置扩展到所有测量时刻
        % ---------------------------------------------------------------

        beacon_i = repmat( ...
            beacon_pos(i,:), ...
            sample_num, ...
            1);
        % ---------------------------------------------------------------
        % 数据格式
        %
        % 1 time
        % 2 slant range
        % 3 horizontal range
        % 4 beacon latitude
        % 5 beacon longitude
        % 6 beacon height
        % --------------------------------------------------------------
        range_beacon{i} = [
            truth(:,2), ...
            slant_range, ...
            horizontal_range, ...
            beacon_i
        ];

    end
    %% --------------------------------------------------------------------
    % 输出辅助数据
    % ---------------------------------------------------------------------
    info = struct();
    info.beacon_num = ...
        beacon_num;


    info.beacon_xyz = ...
        beacon_xyz;


    info.beacon_pos = ...
        beacon_pos;


    info.beacon_deg = ...
        beacon_deg;


    info.trajectory_xyz = ...
        trajectory_xyz;


    info.horizontal_distances = ...
        horizontal_distances;


    info.slant_distances = ...
        slant_distances;

end
function save_static_beacon_range( ...
    range_beacon, ...
    output_dir, ...
    file_prefix)
%SAVE_STATIC_BEACON_RANGE
%
% 将静态信标测距数据分别保存为：
%
% range_static_1.txt
% range_static_2.txt
% ...
%
%
% 输入：
%
% range_beacon
%   generate_static_beacon_range生成的cell
%
% output_dir
%   输出目录
%
% file_prefix
%   文件名前缀
    %% --------------------------------------------------------------------
    % 默认参数
    % ---------------------------------------------------------------------
    if nargin < 3 || isempty(file_prefix)

        file_prefix = ...
            'range_static';
    end
    %% --------------------------------------------------------------------
    % 创建输出文件夹
    % ---------------------------------------------------------------------
    if ~exist(output_dir, 'dir')

        mkdir(output_dir);

    end
    beacon_num = ...
        numel(range_beacon);
    %% --------------------------------------------------------------------
    % 保存
    % ---------------------------------------------------------------------
    for i = 1:beacon_num

        output_file = fullfile( ...
            output_dir, ...
            sprintf( ...
                '%s_%d.txt', ...
                file_prefix, ...
                i));
        try
            writematrix( ...
                range_beacon{i}, ...
                output_file, ...
                'Delimiter', ...
                ' ');
            fprintf( ...
                '信标 %d 距离信息已写入：%s\n', ...
                i, ...
                output_file);
        catch ME

            error( ...
                '信标 %d 距离文件写入失败：%s', ...
                i, ...
                ME.message);
        end
    end
end
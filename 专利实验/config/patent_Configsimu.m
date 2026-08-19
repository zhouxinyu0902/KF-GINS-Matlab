function cfg = patent_Configsimu(require_truth)
%PATENT_CONFIGSIMU 专利仿真的路径、数据生成和滤波参数。

    if nargin < 1
        require_truth = true;
    end

    param = Param();
    glvs

    %% 路径
    config_dir = fileparts(mfilename('fullpath'));
    cfg.topicdir = fileparts(config_dir);
    cfg.projectroot = fileparts(cfg.topicdir);
    cfg.dataroot = fullfile(cfg.projectroot, 'data', 'patent');
    cfg.inputfolder = fullfile(cfg.dataroot, 'input');
    cfg.outputfolder = fullfile(cfg.dataroot, 'output');
    cfg.legacyoutputfolder = fullfile(cfg.topicdir, 'output');
    cfg.figurefolder = fullfile(cfg.outputfolder, 'figures');
    cfg.simfigurefolder = fullfile(cfg.figurefolder, 'simulation');
    cfg.evalfigurefolder = fullfile(cfg.figurefolder, 'evaluation');

    cfg.imufilepath = fullfile(cfg.inputfolder, 'line-imu.nav');
    cfg.rangefile1path = fullfile(cfg.inputfolder, 'line-range1.nav');
    cfg.rangefile2path = fullfile(cfg.inputfolder, 'line-range2.nav');
    cfg.rangefile3path = fullfile(cfg.inputfolder, 'line-range3.nav');
    cfg.truthpath = fullfile(cfg.inputfolder, 'line-truth.nav');

    %% 实验控制
    cfg.starttime = 0;
    cfg.endtime = 7200;
    % 一次运行纯惯导、三个固定单信标和按 1-2-3 轮换的三信标方案。
    cfg.experiment.run_pure_ins = true;
    cfg.experiment.run_single_beacons = true;
    cfg.experiment.run_rotating_beacons = true;
    cfg.experiment.single_beacon_ids = [1, 2, 3];
    cfg.experiment.rotation_sequence = [1, 2, 3];

    % 测距间隔，单位为 min；可设置多个值，例如 [1, 2, 4]。
    cfg.experiment.range_intervals_min = 4;
    cfg.experiment.output_prefix = 'nav';
    cfg.randomseed = 1;

    % 注入到仿真量测的噪声，与滤波采用的量测标准差分别记录。
    cfg.measurement.range_noise_std = 2;
    cfg.measurement.depth_noise_std = 0.2;
    cfg.filter.range_std = 3;
    cfg.filter.depth_std = 0.4;

    %% Figure export
    cfg.figure.save_fig = true;
    cfg.figure.save_png = true;
    cfg.figure.png_resolution = 300;

    %% 仿真几何与 IMU 参数
    cfg.sim.sample_interval = 0.01;
    cfg.sim.beacon_origin_deg = [17.574, 117.7900, 0];
    cfg.sim.layout_m = [0, 0, 0;
                        10, 10*sqrt(3), 0;
                        20, 0, 0;
                        0, 5*sqrt(3), 0;
                        7, 10, 0] * 1000 / 4;
    cfg.sim.rotation_deg = 0;
    cfg.sim.start_point_index = 5;
    cfg.sim.initial_heading_deg = -95;
    cfg.sim.imu_error.eb = 0.027;
    cfg.sim.imu_error.db = 15;
    cfg.sim.imu_error.web = 0.003;
    cfg.sim.imu_error.wdb = 0.03 * 1e5 / 3600;

    %% 初始信息
    if ~require_truth
        return;
    end
    if ~isfile(cfg.truthpath)
        error('未找到专利仿真真值文件：%s。请先运行 generate_simulation_data.m。', cfg.truthpath);
    end
    pva0 = load(cfg.truthpath);
    avp0 = pvaNED2ENU(pva0(1, :));

    avperr = avperrset([0.005; 0.005; 0.03] * 60, 0.1, 1);
    avp00 = avpadderr(avp0, avperr);
    pva00 = avpENU2NED(avp00')';

    cfg.initposstd = avperr(7:9);
    cfg.initvelstd = avperr(4:6);
    cfg.initattstd = avperr(1:3);
    cfg.initpos = avp00(7:9);
    cfg.initvel = pva00(6:8);
    cfg.initatt = pva00(9:11) * param.D2R;

    %% 滤波参数
    cfg.initgyrbias = [0; 0; 0];
    cfg.initaccbias = [0; 0; 0];
    cfg.initgyrscale = [0; 0; 0];
    cfg.initaccscale = [0; 0; 0];

    cfg.initgyrbiasstd = [0.005; 0.005; 0.005];
    cfg.initaccbiasstd = [7; 7; 7];
    cfg.initgyrscalestd = [5; 5; 5];
    cfg.initaccscalestd = [10; 10; 10];

    cfg.gyrarw = 0.0003;
    cfg.accvrw = 1e-6;
    cfg.gyrbiasstd = 0.005;
    cfg.accbiasstd = 7;
    cfg.gyrscalestd = 5;
    cfg.accscalestd = 10;
    cfg.corrtime = 4;

    cfg.antlever = [0; 0; 0];
    cfg.odolever = [0; 0; 0];
    cfg.installangle = [0; 0; 0];

    %% 单位转换
    cfg.initgyrbias = cfg.initgyrbias * param.D2R / 3600;
    cfg.initaccbias = cfg.initaccbias * 1e-5;
    cfg.initgyrscale = cfg.initgyrscale * 1e-6;
    cfg.initaccscale = cfg.initaccscale * 1e-6;
    cfg.initgyrbiasstd = cfg.initgyrbiasstd * param.D2R / 3600;
    cfg.initaccbiasstd = cfg.initaccbiasstd * 1e-5;
    cfg.initgyrscalestd = cfg.initgyrscalestd * 1e-6;
    cfg.initaccscalestd = cfg.initaccscalestd * 1e-6;

    cfg.gyrarw = cfg.gyrarw * param.D2R / 60;
    cfg.accvrw = cfg.accvrw / 60;
    cfg.gyrbiasstd = cfg.gyrbiasstd * param.D2R / 3600;
    cfg.accbiasstd = cfg.accbiasstd * 1e-5;
    cfg.gyrscalestd = cfg.gyrscalestd * 1e-6;
    cfg.accscalestd = cfg.accscalestd * 1e-6;
    cfg.corrtime = cfg.corrtime * 3600;

    cfg.installangle = cfg.installangle * param.D2R;
    cfg.cbv = euler2dcm(cfg.installangle);
end

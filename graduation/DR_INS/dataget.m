close all
clc
glvs
rng(1)
% 鐢ㄤ簬鑾峰彇鏁版嵁
%% ============================================================
%  DVL + 缃楃洏 + 璺濈 鏁版嵁鐢熸垚鑴氭湰
%  鍧愭爣绯伙細NED锛孨/E 鍗曚綅 m锛孌 鍚戜笅涓烘
% ============================================================

%% -------------------- 鍩烘湰鍙傛暟 --------------------
cfg.ts = 0.5;              % 閲囨牱鍛ㄦ湡
cfg.v  = 1;              % AUV閫熷害 m/s
cfg.depth0 = 3893.066;     % 鍒濆娣卞害锛孌涓烘锛屽崟浣峬

% PSINS涓殑楂樺害h鍚戜笂涓烘锛屽洜姝ゆ繁娴烽珮搴︿负璐?
deg = pi/180;
cfg.lat0 = 30 * deg ;
cfg.lon0 = 120 * deg ;
cfg.h0   = -cfg.depth0 ;

% NED鍘熺偣锛氭按骞充綅缃彇AUV鍒濆浣嶇疆锛岄珮搴﹀彇娴烽潰h=0
cfg.ned_origin = [cfg.lat0; cfg.lon0; 0];

% 杞ㄨ抗绫诲瀷锛?
% 'line_N', 'line_E', 'square', 'lawnmower', 'circle'
% traj_case = 'lawnmower';
traj_case = 'line_N';
% 淇℃爣甯冨眬锛?
% 'single_side', 'single_collinear', 'single_center',
% 'single_quadrant_NE', 'single_quadrant_NW',
% 'single_quadrant_SE', 'single_quadrant_SW',
% 'four_quadrants',
% 'two_symmetric', 'triangle', 'moving_parallel', 'moving_cross'
% beacon_case = 'single_side';
beacon_case = 'four_quadrants';
% 璺濈绫诲瀷锛?
% 'slant'      锛氭枩璺濓紝浣跨敤 N/E/D
% 'horizontal' 锛氭按骞宠窛绂伙紝鍙娇鐢?N/E
cfg.range_mode = 'horizontal';

% 璺濈瑙傛祴鍛ㄦ湡
cfg.range_period = 1;      % s
cfg.range_std = 5.0;       % m

% 澶氫俊鏍囨祴璺濇柟寮忥細
% 'simultaneous'锛氭瘡涓祴璺濇椂鍒绘墍鏈変俊鏍囬兘鏈夎窛绂?
% 'round_robin' 锛氭瘡涓祴璺濇椂鍒诲彧娴嬩竴涓俊鏍囷紝杞崲
cfg.range_schedule = 'simultaneous';

%% -------------------- 鏋勯€犺建杩?--------------------
[avp0, seg] = build_traj_segment(traj_case, cfg);

trj = trjsimu(avp0, seg.wat, cfg.ts, 1);
avp_ref = trj.avp;
ts = trj.ts;
ll = length(avp_ref);
t = avp_ref(:,end);

%% -------------------- 杞崲涓篘ED鐪熷€?--------------------
truth_ned = avpENU2NED(avp_ref);

%% -------------------- IMU鏁版嵁鐢熸垚 -------------
imuerr = imuerrset(0.001, 1, 0.01, 10);
imu_noised = imuadderr(trj.imu, imuerr);
imu_FRD = imuRFU2FRD(imu_noised);

%% -------------------- DVL鏁版嵁鐢熸垚 --------------------
% DVL璇樊妯″瀷锛氬埢搴﹀洜瀛?+ 甯稿€艰宸?+ 鐧藉櫔澹?
cfg.dvl_scale = 1.004;              % DVL鍒诲害鍥犲瓙锛屼緥濡?.4%
cfg.dvl_noise = 0.001;              % m/s
cfg.dvl_bias  = [0.001; 0.001; 0.001];  % m/s锛屽彲鏍规嵁闇€瑕佹敼

dvl_true = zeros(ll,4);
vxy = zeros(ll,4);

for k = 1:ll
    cnb = a2mat(avp_ref(k,1:3));
    vb = cnb' * avp_ref(k,4:6)';     % 瀵艰埅绯婚€熷害杞綋鍧愭爣绯?
    dvl_true(k,1:3) = vb';
end

dvl_true(:,4) = t;

vxy(:,1:3) = dvl_true(:,1:3) * cfg.dvl_scale ...
    + repmat(cfg.dvl_bias', ll, 1) ...
    + cfg.dvl_noise * randn(ll,3);
vxy(:,4) = t;
myfigurestartup(12,4,'zxy');
for i = 1:3
    subplot(1,3,i)
    hold on
    plot(avp_ref(:,end),vxy(:,i))
    plot(avp_ref(:,end),dvl_true(:,i))
end
%% -------------------- 缃楃洏鏁版嵁鐢熸垚 --------------------
% pitch / roll璇樊杈冨皬锛寉aw璇樊杈冨ぇ
cfg.pr_bias_deg = 0.02;
cfg.pr_noise_deg = 0.02;
cfg.yaw_noise_deg = 0.05;

cfg.yaw_bias_deg = 0.5;
yaw_bias_env = getenv('DR_INS_YAW_BIAS_DEG');
if ~isempty(yaw_bias_env)
    cfg.yaw_bias_deg = str2double(yaw_bias_env);
    if isnan(cfg.yaw_bias_deg)
        error('Invalid DR_INS_YAW_BIAS_DEG: %s', yaw_bias_env);
    end
end
const_pitch_roll = cfg.pr_bias_deg * deg;
const_yaw = cfg.yaw_bias_deg * deg;

compass = zeros(ll,4);
compass(:,1) = avp_ref(:,1) + const_pitch_roll ...
    + cfg.pr_noise_deg * deg * randn(ll,1);
compass(:,2) = avp_ref(:,2) + const_pitch_roll ...
    + cfg.pr_noise_deg * deg * randn(ll,1);
compass(:,3) = avp_ref(:,3) + const_yaw ...
    + cfg.yaw_noise_deg * deg * randn(ll,1);
compass(:,4) = t;
%  瀵规瘮鐪熷疄缃楃洏鏁版嵁鍜岄噺娴嬫暟鎹?

myfigurestartup(12,4,'zxy');
for i = 1:3
    subplot(1,3,i)
    hold on
    plot(avp_ref(:,end),r2d(compass(:,i)))
    plot(avp_ref(:,end),r2d(avp_ref(:,i)))
end
%% -------------------- 娣卞害璁℃暟鎹敓鎴?--------------------
cfg.depth_std = 0.4;       % m
ned_ref = lla2ned_local(avp_ref(:,7:9), cfg.ned_origin);
depth_true = ned_ref(:,3); % NED涓璂涓烘锛岀洿鎺ヤ綔涓烘繁搴?
depth = depth_true + cfg.depth_std * randn(ll,1);

%% -------------------- 淇℃爣甯冨眬涓庤窛绂荤敓鎴?--------------------
beacons = build_beacons_ned(beacon_case, ned_ref, t, cfg);

[range_true, range_meas, horiz_true, horiz_meas, beacon_obs, range_idx] = ...
    simulate_beacon_range_data(ned_ref, t, beacons, cfg);


data_name = ['data_', traj_case, '_', beacon_case];
data_suffix = getenv('DR_INS_DATA_SUFFIX');
if ~isempty(data_suffix)
    data_name = [data_name, '_', data_suffix];
end
outdir = fullfile('D:\Github\KF-GINS-Matlab\graduation\DR_INS\input', data_name);
if ~exist(outdir,'dir')
    mkdir(outdir)
end
%% -------------------- 鐢诲浘妫€鏌?--------------------
fig1 = myfigurestartup(4,4,'zxy');
plot(ned_ref(:,2), ned_ref(:,1))
hold on
plot(ned_ref(1,2), ned_ref(1,1),'diamond','DisplayName','start')
axis equal
grid on
xlabel('East / m')
ylabel('North / m')
title(['Trajectory in NED: ', traj_case])
% 鐢讳俊鏍?
M = size(beacons.ned_time,3);
for m = 1:M
    bm = squeeze(beacons.ned_time(:,:,m));
    if beacons.is_moving
        plot(bm(:,2), bm(:,1), '--', 'LineWidth', 1.0)
        plot(bm(1,2), bm(1,1), 'o', 'MarkerSize', 8)
    else
        plot(bm(1,2), bm(1,1), 'p', 'MarkerSize', 12, 'LineWidth', 1.5)
        text(bm(1,2)+500, bm(1,1)-100, sprintf("B%d",m)) 
    end
end
legend('AUV trajectory', 'Beacon / beacon trajectory')
exportgraphics(fig1,outdir + "/trj_bea.png", "Resolution",600)

%% -------------------- 淇濆瓨涓?txt 鏁版嵁 --------------------

if ~exist(outdir, 'dir')
    mkdir(outdir);
end

% 1. 鍙傝€冪湡鍊兼暟鎹?reference.txt
writematrix(truth_ned, fullfile(outdir, 'reference.txt'), ...
    'Delimiter', 'tab');

% 2. IMU鏁版嵁 imu.txt
writematrix(imu_FRD, fullfile(outdir, 'imu.txt'), ...
    'Delimiter', 'tab');

% 3. DVL鏁版嵁 dvl.txt
dvl_txt = [ ...
    vxy(:,4), ...
    vxy(:,1:3) ...
    ];

writematrix(dvl_txt, fullfile(outdir, 'dvl.txt'), ...
    'Delimiter', 'tab');

% 4. 缃楃洏鏁版嵁 compass.txt
compass_txt = [ ...
    compass(:,4), ...
    compass(:,1:3) ...
    ];
writematrix(compass_txt, fullfile(outdir, 'compass.txt'), ...
    'Delimiter', 'tab');

% 5. 娣卞害璁℃暟鎹?depth.txt
depth_txt = [  t, depth ];
writematrix(depth_txt, fullfile(outdir, 'depth.txt'), ...
    'Delimiter', 'tab');

% 6. 銆愪慨鏀归儴鍒嗐€戜俊鏍?璺濈鏁版嵁鍒嗘祦淇濆瓨 (濡?beacon_1.txt, beacon_2.txt ...)
% 鍗曚釜鏂囦欢鍐呯殑鏍煎紡渚濈劧淇濇寔鍘熸牱锛?
% time  slant_range  horizontal_range  beacon_lat  beacon_lon  beacon_h
M = size(beacons.ned_time, 3); % 鑾峰彇瀹為檯淇℃爣鎬绘暟
for m = 1:M
    % 绛涢€夊嚭灞炰簬褰撳墠淇℃爣 m 鐨勮娴嬭
    idx = (beacon_obs(:, 2) == m);
    sub_obs = beacon_obs(idx, :);
    
    % 鍓旈櫎涓存椂寮曞叆鐨勭浜屽垪 (beacon_id)锛屾仮澶嶅師鐢?6 鍒楃函鏁版嵁鏍煎紡
    sub_obs(:, 2) = []; 
    
    % 鍔ㄦ€佸懡鍚嶅苟鍐欏叆瀵瑰簲鐨勬枃鏈枃浠?
    filename = sprintf('beacon_%d.txt', m);
    writematrix(sub_obs, fullfile(outdir, filename), 'Delimiter', 'tab');
end



% 7. 閰嶇疆璇存槑 config.txt
fid = fopen(fullfile(outdir, 'config.txt'), 'w');

fprintf(fid, 'Trajectory case: %s\n', traj_case);
fprintf(fid, 'Beacon case: %s\n', beacon_case);
fprintf(fid, 'Sampling time ts: %.6f s\n', cfg.ts);
fprintf(fid, 'AUV velocity: %.6f m/s\n', cfg.v);
fprintf(fid, 'Range mode: %s\n', cfg.range_mode);
fprintf(fid, 'Range period: %.6f s\n', cfg.range_period);
fprintf(fid, 'Range std: %.6f m\n', cfg.range_std);
fprintf(fid, 'Range schedule: %s\n', cfg.range_schedule);
fprintf(fid, 'DVL scale factor: %.8f\n', cfg.dvl_scale);
fprintf(fid, 'DVL noise std: %.8f m/s\n', cfg.dvl_noise);
fprintf(fid, 'Depth noise std: %.8f m\n', cfg.depth_std);
fprintf(fid, 'Yaw bias: %.8f deg\n', cfg.yaw_bias_deg);
fprintf(fid, 'Yaw noise std: %.8f deg\n', cfg.yaw_noise_deg);

fprintf(fid, '\nFile format:\n');
fprintf(fid, 'reference.txt: t N E D pitch roll yaw vN vE vD\n');
fprintf(fid, 'imu.txt:       t gyro_x gyro_y gyro_z acc_x acc_y acc_z\n');
fprintf(fid, 'dvl.txt:       t vb_x vb_y vb_z\n');
fprintf(fid, 'compass.txt:   t pitch roll yaw\n');
fprintf(fid, 'depth.txt:     t depth\n');
fprintf(fid, 'beacon_x.txt:  t slant_range horizontal_range beacon_lat beacon_lon beacon_h\n');

fclose(fid);

disp(['TXT data saved to: ', outdir]);


%% ============================================================
%                    Local functions
% ============================================================

function [avp0, seg] = build_traj_segment(traj_case, cfg)
xxx = [];
deg = pi/180;
v = cfg.v;
pos0 = [cfg.lat0; cfg.lon0; cfg.h0];

switch traj_case
    case 'line_N'
        yaw0 = 0 * deg;
        avp0 = [[0;0;yaw0]; [0;0;0]; pos0];
        line_len = 3600;       
        seg = trjsegment(xxx, 'init', 0);
        seg = trjsegment(seg, 'accelerate', 10, xxx, v/10);
        seg = trjsegment(seg, 'uniform', line_len/v);
    case 'line_E'
        yaw0 = 90 * deg;
        avp0 = [[0;0;yaw0]; [0;0;0]; pos0];
        line_len = 3600;
        seg = trjsegment(xxx, 'init', 0);
        seg = trjsegment(seg, 'accelerate', 10, xxx, v/10);
        seg = trjsegment(seg, 'uniform', line_len/v);
    case 'square'
        yaw0 = 90 * deg;
        avp0 = [[0;0;yaw0]; [0;0;0]; pos0];
        side = 1000;           
        seg = trjsegment(xxx, 'init', 0);
        seg = trjsegment(seg, 'accelerate', 10, xxx, v/10);
        for k = 1:4
            seg = trjsegment(seg, 'uniform', side/v);
            seg = trjsegment(seg, 'turnleft', 90, 1);
        end
    case 'lawnmower'
        yaw0 = 90 * deg;
        avp0 = [[0;0;yaw0]; [0;0;0]; pos0];
        track_len = 3600;      
        spacing   = 200;       
        n_line    = 6;         
        seg = trjsegment(xxx, 'init', 0);
        seg = trjsegment(seg, 'uniform', 20);
        seg = trjsegment(seg, 'accelerate', 10, xxx, v/10);
        seg = trjsegment(seg, 'uniform', track_len/v);
        for k = 1:n_line-1
            if mod(k,2) == 1
                seg = trjsegment(seg, 'turnleft', 90, 1);
                seg = trjsegment(seg, 'uniform', spacing/v);
                seg = trjsegment(seg, 'turnleft', 90, 1);
            else
                seg = trjsegment(seg, 'turnright', 90, 1);
                seg = trjsegment(seg, 'uniform', spacing/v);
                seg = trjsegment(seg, 'turnright', 90, 1);
            end
            seg = trjsegment(seg, 'uniform', track_len/v);
        end
    case 'circle'
        yaw0 = 90 * deg;
        avp0 = [[0;0;yaw0]; [0;0;0]; pos0];
        seg = trjsegment(xxx, 'init', 0);
        seg = trjsegment(seg, 'accelerate', 10, xxx, v/10);
        seg = trjsegment(seg, 'uniform', 100);
        seg = trjsegment(seg, 'turnleft', 360, 0.2);
    otherwise
        error('Unknown traj_case.')
end
end


function ned = lla2ned_local(lla, origin)
lat = lla(:,1); lon = lla(:,2); h = lla(:,3);
lat0 = origin(1); lon0 = origin(2); h0 = origin(3);
a = 6378137.0; f = 1 / 298.257223563; e2 = f * (2 - f);
sin_lat0 = sin(lat0);
RM = a * (1 - e2) / (1 - e2 * sin_lat0^2)^(3/2);
RN = a / sqrt(1 - e2 * sin_lat0^2);
dLat = lat - lat0; dLon = lon - lon0; dH = h - h0;
N = dLat * (RM + h0); E = dLon * (RN + h0) * cos(lat0); D = -dH;
ned = [N, E, D];
end


function beacons = build_beacons_ned(beacon_case, ned_ref, t, cfg)
ll = length(t);
Nmin = min(ned_ref(:,1)); Nmax = max(ned_ref(:,1));
Emin = min(ned_ref(:,2)); Emax = max(ned_ref(:,2));
Nc = 0.5 * (Nmin + Nmax); Ec = 0.5 * (Emin + Emax); Dc = median(ned_ref(:,3));
area_scale = max([Nmax-Nmin, Emax-Emin, 500]);
offset = 0.4 * area_scale;
D_surface = 10;
beacons = struct; beacons.case = beacon_case; beacons.is_moving = false;

switch beacon_case
    case 'single_side'
        B = [Nc, Emax + offset, D_surface];
    case 'single_collinear'
        B = [Nc, Emin - offset, D_surface];
    case 'single_center'
        B = [Nc, Ec, D_surface];
    case 'single_quadrant_NE'
        B = [Nmax + offset, Emax + offset, D_surface];
    case 'single_quadrant_NW'
        B = [Nmax + offset, Emin - offset, D_surface];
    case 'single_quadrant_SE'
        B = [Nmin - offset, Emax + offset, D_surface];
    case 'single_quadrant_SW'
        B = [Nmin - offset, Emin - offset, D_surface];
    case 'four_quadrants'
        offset = 3600;
        % B = [Nmax + offset, Emax + offset, D_surface;
        %     Nmax + offset, Emin - offset, D_surface;
        %     Nmin - offset, Emax + offset, D_surface;
        %     Nmin - offset, Emin - offset, D_surface];
        B = [Nmin + offset, Emin + offset, D_surface;
            Nmin + offset, Emin - offset, D_surface;
            Nmin - offset, Emin + offset, D_surface;
            Nmin - offset, Emin - offset, D_surface];

    case 'two_symmetric'
        B = [Nc, Ec - offset, D_surface; Nc, Ec + offset, D_surface];
    case 'triangle'
        B = [Nmin - offset, Emin - offset, D_surface;
            Nmax + offset, Emin - offset, D_surface;
            Nc,           Emax + offset, D_surface];
    case 'moving_parallel'
        beacons.is_moving = true;
        Btime = zeros(ll,3,1);
        Btime(:,1,1) = ned_ref(:,1); Btime(:,2,1) = ned_ref(:,2) + offset; Btime(:,3,1) = D_surface * ones(ll,1);
        beacons.ned_time = Btime; return
    case 'moving_cross'
        beacons.is_moving = true;
        Btime = zeros(ll,3,1);
        Btime(:,1,1) = linspace(Nmin - offset, Nmax + offset, ll)';
        Btime(:,2,1) = linspace(Emax + offset, Emin - offset, ll)';
        Btime(:,3,1) = D_surface * ones(ll,1);
        beacons.ned_time = Btime; return
    otherwise
        error('Unknown beacon_case.')
end

M = size(B,1);
Btime = zeros(ll,3,M);
for m = 1:M
    Btime(:,:,m) = repmat(B(m,:), ll, 1);
end
beacons.ned = B; beacons.ned_time = Btime;
end


function [range_true, range_meas, horiz_true, horiz_meas, beacon_obs, range_idx] = ...
    simulate_beacon_range_data(ned_ref, t, beacons, cfg)

ll = length(t);
M = size(beacons.ned_time, 3);

range_true = zeros(ll, M);   range_meas = nan(ll, M);
horiz_true = zeros(ll, M);   horiz_meas = nan(ll, M);

step = max(1, round(cfg.range_period / cfg.ts));
range_idx = 1:step:ll;
beacon_obs = [];

for m = 1:M
    Bm_all = squeeze(beacons.ned_time(:,:,m));
    dN = ned_ref(:,1) - Bm_all(:,1);
    dE = ned_ref(:,2) - Bm_all(:,2);
    dD = ned_ref(:,3) - Bm_all(:,3);
    range_true(:,m) = sqrt(dN.^2 + dE.^2 + dD.^2);
    horiz_true(:,m) = sqrt(dN.^2 + dE.^2);
end

for kk = 1:length(range_idx)
    k = range_idx(kk);
    switch cfg.range_schedule
        case 'simultaneous'
            active_beacons = 1:M;
        case 'round_robin'
            active_beacons = mod(kk-1, M) + 1;
        otherwise
            error('Unknown range_schedule.')
    end

    for m = active_beacons
        Bm = squeeze(beacons.ned_time(k,:,m));
        dN = ned_ref(k,1) - Bm(1);
        dE = ned_ref(k,2) - Bm(2);
        dD = ned_ref(k,3) - Bm(3);

        slant_true_k = sqrt(dN^2 + dE^2 + dD^2);
        horiz_true_k = sqrt(dN^2 + dE^2);
        slant_meas_k = slant_true_k + cfg.range_std * randn;
        horiz_meas_k = horiz_true_k + cfg.range_std * randn;

        range_meas(k,m) = slant_meas_k;
        horiz_meas(k,m) = horiz_meas_k;

        beacon_lla = ned2lla_local(Bm, cfg.ned_origin);
        beacon_lat = beacon_lla(1);
        beacon_lon = beacon_lla(2);
        beacon_h   = beacon_lla(3);

        % 銆愪慨鏀归儴鍒嗐€戝湪绗簩鍒楁彃鍏ヤ簡淇℃爣鏍囪瘑 m锛屼究浜庡閮ㄥ垎娴佷繚瀛?
        beacon_obs = [beacon_obs;
            t(k), ...
            m, ...   
            slant_meas_k, ...
            horiz_meas_k, ...
            beacon_lat, ...
            beacon_lon, ...
            beacon_h];
    end
end
end

function lla = ned2lla_local(ned, origin)
N = ned(1); E = ned(2); D = ned(3);
lat0 = origin(1); lon0 = origin(2); h0 = origin(3);
a = 6378137.0; f = 1 / 298.257223563; e2 = f * (2 - f);
sin_lat0 = sin(lat0);
RM = a * (1 - e2) / (1 - e2 * sin_lat0^2)^(3/2);
RN = a / sqrt(1 - e2 * sin_lat0^2);
dLat = N / (RM + h0); dLon = E / ((RN + h0) * cos(lat0)); dH = -D;
lat = lat0 + dLat; lon = lon0 + dLon; h = h0 + dH;
lla = [lat, lon, h];
end

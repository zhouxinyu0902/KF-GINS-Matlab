%% Exper_LOBO_Navigation
% Evaluate measured fixed-beacon range aiding against strict LOBO references.
% Only the DR baseline and the proposed 5-state ES-EKF are run.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
sum_root = fileparts(script_dir);
mytest_root = fileparts(sum_root);
psins_root = fileparts(mytest_root);
repo_root = fileparts(psins_root);

addpath(genpath(fullfile(psins_root, 'base')));
addpath(fullfile(mytest_root, '00_all_func'));
addpath(fullfile(sum_root, 'func_1'));
glvs;

output_root = fullfile(repo_root, 'data', 'psins', 'LOBO');
reference_file = fullfile(output_root, 'reference', 'lobo_reference_data.mat');
navigation_dir = fullfile(output_root, 'navigation');
table_dir = fullfile(output_root, 'tables');
figure_dir = fullfile(output_root, 'figures');
log_dir = fullfile(output_root, 'logs');
ensure_folder(navigation_dir);
ensure_folder(table_dir);
ensure_folder(figure_dir);
ensure_folder(log_dir);

log_file = fullfile(log_dir, 'Exper_LOBO_Navigation.log');
if exist(log_file, 'file'), delete(log_file); end
diary(log_file);
diary_cleanup = onCleanup(@() diary('off')); %#ok<NASGU>

fprintf('LOBO 5-state navigation started: %s\n', datestr(now,31));
assert(exist(reference_file,'file') == 2, ...
    'Run Exper_LOBO_Reference.m first: %s', reference_file);
R = load(reference_file);

source = load(R.metadata.source_file, 'compass', 'vxy');
t = R.t(:);
N = numel(t);
assert(size(source.compass,1) == N && size(source.vxy,1) == N, ...
    'Navigation sensor length mismatch.');

cfg = struct();
cfg.state_dimension = 5;
cfg.filter_update = 'EKF';
cfg.navigation_dt_s = 0.5;
cfg.acoustic_interval_s = 8;
cfg.range_noise_std_m = 5;
cfg.x0 = zeros(5,1);
cfg.dx0 = [0.002; d2r(5); d2r(5); 5/glv.Re; 5/glv.Re];
cfg.vk = [0; d2r(0.08); d2r(0.08); 0; 0];

results = cell(1,4);
summary_rows = repmat(struct(),4,1);
for held_out = 1:4
    fprintf('Fold %d/4: 5-state ES-EKF uses held-out B%d range.\n', ...
        held_out, held_out);
    reference = R.ref_lobo{held_out};
    beacon = R.beacon_pos{held_out}(:)';
    measured_slant_range = R.range_clean{held_out}(:);
    results{held_out} = run_fold(reference, source.compass, source.vxy, ...
        R.vehicle_depth, t, beacon, measured_slant_range, cfg);

    dr_lobo = horizontal_error_stats(results{held_out}.avp_dr, reference);
    ekf_lobo = horizontal_error_stats(results{held_out}.avp_ekf, reference);
    dr_full = horizontal_error_stats(results{held_out}.avp_dr, R.original_ref);
    ekf_full = horizontal_error_stats(results{held_out}.avp_ekf, R.original_ref);

    summary_rows(held_out).HeldOutBeacon = held_out;
    summary_rows(held_out).ReferenceBeacons = sprintf('B%s', ...
        strjoin(string(setdiff(1:4,held_out,'stable')), '+B'));
    summary_rows(held_out).EvaluationEpochs = ekf_lobo.N;
    summary_rows(held_out).AcousticUpdates = ...
        nnz(results{held_out}.update_mask);
    summary_rows(held_out).DR_RMSE_LOBO_m = dr_lobo.RMSE_m;
    summary_rows(held_out).DR_Max_LOBO_m = dr_lobo.Max_m;
    summary_rows(held_out).DR_P95_LOBO_m = dr_lobo.P95_m;
    summary_rows(held_out).DR_Final_LOBO_m = dr_lobo.Final_m;
    summary_rows(held_out).EKF_RMSE_LOBO_m = ekf_lobo.RMSE_m;
    summary_rows(held_out).EKF_Max_LOBO_m = ekf_lobo.Max_m;
    summary_rows(held_out).EKF_P95_LOBO_m = ekf_lobo.P95_m;
    summary_rows(held_out).EKF_Final_LOBO_m = ekf_lobo.Final_m;
    summary_rows(held_out).EKF_GainVsDR_Percent = ...
        100*(dr_lobo.RMSE_m-ekf_lobo.RMSE_m)/dr_lobo.RMSE_m;
    summary_rows(held_out).DR_RMSE_OriginalRef_m = dr_full.RMSE_m;
    summary_rows(held_out).EKF_RMSE_OriginalRef_m = ekf_full.RMSE_m;
    summary_rows(held_out).EKF_RMSE_ReferenceShift_m = ...
        ekf_lobo.RMSE_m-ekf_full.RMSE_m;
end

navigation_summary = struct2table(summary_rows);
writetable(navigation_summary, ...
    fullfile(table_dir,'lobo_navigation_5state_summary.csv'));

save(fullfile(navigation_dir,'lobo_navigation_5state.mat'), ...
    'results','navigation_summary','cfg','-v7.3');
%%
plot_navigation_errors(t,R.ref_lobo,results,figure_dir);
plot_navigation_trajectories(R.ref_lobo,results,R.beacon_pos,figure_dir);
plot_rmse_summary(navigation_summary,figure_dir);

completion_file = fullfile(log_dir,'NAVIGATION_COMPLETED.txt');
fid = fopen(completion_file,'w');
assert(fid >= 0,'Cannot create navigation completion marker.');
fprintf(fid,'Completed: %s\n',datestr(now,31));
fprintf(fid,'Filter: 5-state ES-EKF\n');
fprintf(fid,'Update implementation: %s\n',cfg.filter_update);
fprintf(fid,'Summary: %s\n',fullfile(table_dir,'lobo_navigation_5state_summary.csv'));
fclose(fid);

disp(navigation_summary);
fprintf('LOBO 5-state navigation completed: %s\n',datestr(now,31));

%% Local functions
function result = run_fold(reference, compass, vxy, vehicle_depth, t, ...
        beacon, measured_slant_range, cfg)
    glvs;
    N = numel(t);
    valid_ref = all(isfinite(reference(:,7:9)),2);
    first_epoch = find(valid_ref,1,'first');
    assert(~isempty(first_epoch),'LOBO reference contains no valid epoch.');

    pos0 = reference(first_epoch,7:9)';
    dr_baseline = mydr('init',pos0,[0;0;0],cfg.navigation_dt_s);
    dr_filter = mydr('init',pos0,[0;0;0],cfg.navigation_dt_s);
    kf = myekf('init',cfg.navigation_dt_s,cfg.x0,cfg.dx0, ...
        cfg.vk,cfg.range_noise_std_m);

    avp_dr = nan(N,10);
    avp_ekf = nan(N,10);
    update_mask = false(N,1);
    innovation_m = nan(N,1);
    measured_horizontal_range_m = nan(N,1);
    predicted_horizontal_range_m = nan(N,1);
    state_record = nan(N,5);
    covariance_diagonal = nan(N,5);

    for k = first_epoch:N
        dr_baseline = mydr('update',dr_baseline,vehicle_depth(k), ...
            compass(k,3),vxy(k,1:2));
        dr_filter = mydr('update',dr_filter,vehicle_depth(k), ...
            compass(k,3),vxy(k,1:2));
        avp_dr(k,:) = [dr_baseline.avp',t(k)];

        kf = myekf('fk',kf,dr_filter);
        kf = myekf('algo',kf,'T');

        is_update_epoch = t(k) > 0 && ...
            abs(mod(t(k),cfg.acoustic_interval_s)) < 1e-8;
        raw_range = measured_slant_range(k);
        dz = vehicle_depth(k)-beacon(3);
        valid_range = isfinite(raw_range) && raw_range > abs(dz) && ...
            raw_range > 500 && raw_range < 8000;
        if is_update_epoch && valid_range
            r_meas = sqrt(max(raw_range^2-dz^2,0));
            predicted_slant = RCompu(dr_filter.pos',beacon);
            r_pred = sqrt(max(predicted_slant^2-dz^2,0));
            dr_filter.beacon = beacon;
            kf.r_dr = r_pred;
            kf.yk = r_pred-r_meas;
            kf = myekf('hk',kf,dr_filter,'range');
            kf = myekf('algo',kf,'M',cfg.filter_update);

            innovation_m(k) = kf.yk-kf.ykk_1;
            measured_horizontal_range_m(k) = r_meas;
            predicted_horizontal_range_m(k) = r_pred;
            update_mask(k) = true;

            dr_filter.pos(1:2) = dr_filter.pos(1:2)-kf.xk(end-1:end);
            kf.xk(end-1:end) = 0;
        end

        avp_ekf(k,:) = [dr_filter.att;dr_filter.vn;dr_filter.pos;t(k)]';
        state_record(k,:) = kf.xk';
        covariance_diagonal(k,:) = diag(kf.Pxk)';
    end

    result = struct();
    result.avp_dr = avp_dr;
    result.avp_ekf = avp_ekf;
    result.update_mask = update_mask;
    result.innovation_m = innovation_m;
    result.measured_horizontal_range_m = measured_horizontal_range_m;
    result.predicted_horizontal_range_m = predicted_horizontal_range_m;
    result.state = state_record;
    result.covariance_diagonal = covariance_diagonal;
    result.first_epoch = first_epoch;
end

function stats = horizontal_error_stats(estimate,reference)
    valid = all(isfinite(estimate(:,7:8)),2) & ...
        all(isfinite(reference(:,7:8)),2);
    if ~any(valid)
        stats = struct('N',0,'RMSE_m',NaN,'Max_m',NaN, ...
            'P95_m',NaN,'Mean_m',NaN,'Final_m',NaN);
        return;
    end
    lat = reference(valid,7);
    dN = (estimate(valid,7)-lat)*6378137.0;
    dE = (estimate(valid,8)-reference(valid,8)).*(6378137.0*cos(lat));
    e = hypot(dE,dN);
    stats = struct('N',numel(e),'RMSE_m',sqrt(mean(e.^2)), ...
        'Max_m',max(e),'P95_m',percentile_local(e,95), ...
        'Mean_m',mean(e),'Final_m',e(end));
end

function plot_navigation_errors(t,refs,results,out_dir)
    fig = figure('Visible','off','Color','w','Position',[100 100 1000 700]);
    tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for j = 1:4
        nexttile;
        e_dr = error_series(results{j}.avp_dr,refs{j});
        e_ekf = error_series(results{j}.avp_ekf,refs{j});
        plot(t,e_dr,'Color',[0.45 0.45 0.45],'LineWidth',1.0, ...
            'DisplayName','DR'); hold on;
        plot(t,e_ekf,'Color',[0.00 0.45 0.74],'LineWidth',1.1, ...
            'DisplayName','5-state ES-EKF');
        grid on; xlabel('Time (s)'); ylabel('Horizontal error (m)');
        title(sprintf('Held-out B%d',j)); legend('Location','best');
    end
    title(tl,'LOBO navigation errors');
    exportgraphics(fig,fullfile(out_dir,'lobo_navigation_errors.png'),'Resolution',240);
    exportgraphics(fig,fullfile(out_dir,'lobo_navigation_errors.pdf'),'ContentType','vector');
    close(fig);
end

function plot_navigation_trajectories(refs,results,beacons,out_dir)
    fig = figure('Visible','off','Color','w','Position',[100 100 1000 750]);
    tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for j = 1:4
        nexttile;
        origin = refs{j}(find(all(isfinite(refs{j}(:,7:8)),2),1),7:8);
        [Er,Nr] = local_xy(refs{j}(:,7:8),origin);
        [Ed,Nd] = local_xy(results{j}.avp_dr(:,7:8),origin);
        [Ee,Ne] = local_xy(results{j}.avp_ekf(:,7:8),origin);
        [Eb,Nb] = local_xy(beacons{j}(1:2),origin);
        plot(Er,Nr,'k-','LineWidth',1.4,'DisplayName','LOBO reference'); hold on;
        plot(Ed,Nd,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0,'DisplayName','DR');
        plot(Ee,Ne,'Color',[0 0.45 0.74],'LineWidth',1.1,'DisplayName','5-state ES-EKF');
        plot(Eb,Nb,'p','MarkerSize',10,'MarkerFaceColor',[0.85 0.33 0.10], ...
            'Color',[0.85 0.33 0.10],'DisplayName',sprintf('B%d',j));
        axis equal; grid on; xlabel('East (m)'); ylabel('North (m)');
        title(sprintf('Held-out B%d',j)); legend('Location','best');
    end
    title(tl,'LOBO trajectories');
    exportgraphics(fig,fullfile(out_dir,'lobo_navigation_trajectories.png'),'Resolution',240);
    exportgraphics(fig,fullfile(out_dir,'lobo_navigation_trajectories.pdf'),'ContentType','vector');
    close(fig);
end

function plot_rmse_summary(T,out_dir)
    fig = figure('Visible','off','Color','w','Position',[100 100 780 520]);
    values = [T.DR_RMSE_LOBO_m,T.EKF_RMSE_LOBO_m];
    bar(values); grid on;
    xticklabels(compose('Held-out B%d',T.HeldOutBeacon));
    ylabel('Horizontal RMSE (m)');
    legend({'DR','5-state ES-EKF'},'Location','best');
    title('LOBO navigation RMSE');
    exportgraphics(fig,fullfile(out_dir,'lobo_navigation_rmse.png'),'Resolution',240);
    exportgraphics(fig,fullfile(out_dir,'lobo_navigation_rmse.pdf'),'ContentType','vector');
    close(fig);
end

function e = error_series(estimate,reference)
    e = nan(size(estimate,1),1);
    valid = all(isfinite(estimate(:,7:8)),2) & all(isfinite(reference(:,7:8)),2);
    lat = reference(valid,7);
    dN = (estimate(valid,7)-lat)*6378137.0;
    dE = (estimate(valid,8)-reference(valid,8)).*(6378137.0*cos(lat));
    e(valid) = hypot(dE,dN);
end

function [E,N] = local_xy(pos,origin)
    E = (pos(:,2)-origin(2))*6378137.0*cos(origin(1));
    N = (pos(:,1)-origin(1))*6378137.0;
end

function value = percentile_local(x,p)
    x = sort(x(isfinite(x)));
    if isempty(x), value = NaN; return; end
    q = 1+(numel(x)-1)*p/100;
    lo = floor(q); hi = ceil(q);
    if lo == hi
        value = x(lo);
    else
        value = x(lo)+(q-lo)*(x(hi)-x(lo));
    end
end

function ensure_folder(path_value)
    if ~exist(path_value,'dir'), mkdir(path_value); end
end

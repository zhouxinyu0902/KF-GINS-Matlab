%% Exper_LOBO_Reference
% Construct four leave-one-beacon-out (LOBO) sea-trial references.
%
% For fold j, beacon j is excluded before the LBL position solution is
% formed.  The resulting three-beacon/depth fixes are then fused with
% OCTANS heading and DVL velocity using the same reference-filter settings
% as AcousticDeadR.m.  No simulated, moving, or virtual beacon is used.

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

input_file = fullfile(repo_root, 'data', 'psins', 'data_1', 'output', ...
    'deep-sea_optimized.mat');
output_root = fullfile(repo_root, 'data', 'psins', 'LOBO');
reference_dir = fullfile(output_root, 'reference');
table_dir = fullfile(output_root, 'tables');
figure_dir = fullfile(output_root, 'figures');
log_dir = fullfile(output_root, 'logs');
ensure_folder(reference_dir);
ensure_folder(table_dir);
ensure_folder(figure_dir);
ensure_folder(log_dir);

log_file = fullfile(log_dir, 'Exper_LOBO_Reference.log');
if exist(log_file, 'file'), delete(log_file); end
diary(log_file);
diary_cleanup = onCleanup(@() diary('off')); %#ok<NASGU>

fprintf('LOBO reference construction started: %s\n', datestr(now, 31));
fprintf('Input: %s\n', input_file);
fprintf('Output: %s\n', output_root);

required = {'LBL_out','BCN','RNG','octans','vxy','depther','compass','avp_LBL_DR'};
S = load(input_file, required{:});
for i = 1:numel(required)
    assert(isfield(S, required{i}), 'Missing variable in input MAT file: %s', required{i});
end

t = S.LBL_out.t(:);
N = numel(t);
assert(size(S.octans,1) == N && size(S.vxy,1) == N && ...
    numel(S.depther) == N && size(S.compass,1) == N, ...
    'Sensor arrays do not share the LBL time axis.');
assert(numel(S.BCN) == 4 && numel(S.RNG) == 4, ...
    'Exactly four fixed-beacon channels are required.');

vehicle_depth = -S.depther(:);  % Height convention used by the existing code.
beacon_pos = S.BCN;
range_clean = S.RNG;

fprintf('Solving all-four-beacon reference using the new common solver...\n');
[lbl_full, solve_full] = solve_depth_constrained_lbl( ...
    beacon_pos, range_clean, vehicle_depth, t, 1:4, 3);
[ref_full_new, fusion_full] = fuse_lbl_octans_dvl( ...
    lbl_full, S.octans, S.vxy, vehicle_depth, t);

ref_lobo = cell(1,4);
lbl_lobo = cell(1,4);
solve_lobo = cell(1,4);
fusion_lobo = cell(1,4);
for held_out = 1:4
    included = setdiff(1:4, held_out, 'stable');
    fprintf('Fold %d/4: reference uses B%s; held-out input is B%d.\n', ...
        held_out, sprintf('%d', included), held_out);
    [lbl_lobo{held_out}, solve_lobo{held_out}] = ...
        solve_depth_constrained_lbl(beacon_pos, range_clean, ...
        vehicle_depth, t, included, 3);
    [ref_lobo{held_out}, fusion_lobo{held_out}] = ...
        fuse_lbl_octans_dvl(lbl_lobo{held_out}, S.octans, S.vxy, ...
        vehicle_depth, t);
end

original_ref = S.avp_LBL_DR;
assert(size(original_ref,1) == N, 'Original reference length mismatch.');
replication = horizontal_error_stats(ref_full_new, original_ref);
fprintf(['New all-four reference versus original avp_LBL_DR: ' ...
    'RMSE %.3f m, max %.3f m, P95 %.3f m.\n'], ...
    replication.RMSE_m, replication.Max_m, replication.P95_m);

rows = repmat(struct(), 4, 1);
for held_out = 1:4
    ref_diff = horizontal_error_stats(ref_lobo{held_out}, ref_full_new);
    [held_residual, held_stats] = withheld_range_residual( ...
        ref_lobo{held_out}, range_clean{held_out}, beacon_pos{held_out});
    solve_diag = solve_lobo{held_out};
    rows(held_out).HeldOutBeacon = held_out;
    rows(held_out).ReferenceBeacons = sprintf('B%s', ...
        strjoin(string(setdiff(1:4, held_out, 'stable')), '+B'));
    rows(held_out).TotalEpochs = N;
    rows(held_out).ValidLBLFixes = nnz(solve_diag.valid_fix);
    rows(held_out).ValidLBLPercent = 100 * mean(solve_diag.valid_fix);
    rows(held_out).MedianLBLResidualRMSE_m = ...
        median(solve_diag.residual_rmse_m, 'omitnan');
    rows(held_out).P95LBLResidualRMSE_m = ...
        percentile_local(solve_diag.residual_rmse_m, 95);
    rows(held_out).MedianGeometryCondition = ...
        median(solve_diag.geometry_condition, 'omitnan');
    rows(held_out).LOBO_vs_Full_RMSE_m = ref_diff.RMSE_m;
    rows(held_out).LOBO_vs_Full_Max_m = ref_diff.Max_m;
    rows(held_out).LOBO_vs_Full_P95_m = ref_diff.P95_m;
    rows(held_out).HeldOutRangeBias_m = held_stats.Mean_m;
    rows(held_out).HeldOutRangeStd_m = held_stats.Std_m;
    rows(held_out).HeldOutRangeRMSE_m = held_stats.RMSE_m;
    rows(held_out).HeldOutRangeP95Abs_m = held_stats.P95Abs_m;
    solve_lobo{held_out}.held_out_range_residual_m = held_residual;
end

reference_summary = struct2table(rows);
writetable(reference_summary, fullfile(table_dir, 'lobo_reference_summary.csv'));

replication_table = struct2table(replication);
writetable(replication_table, ...
    fullfile(table_dir, 'lobo_full_reference_replication.csv'));

metadata = struct();
metadata.created = datestr(now, 31);
metadata.source_file = input_file;
metadata.method = ['Strict LOBO: held-out range excluded before LBL solve; ' ...
    'all three retained ranges required for a direct LBL fix.'];
metadata.range_preprocessing = ['Uses LBL_out.RNG produced by the manuscript ' ...
    'preprocessing, followed by the same 11-point median filter used in the ' ...
    'depth-constrained LBL solver.'];
metadata.shared_sensor_caveat = ['The LOBO reference and evaluated navigation ' ...
    'still share DVL and depth inputs; LOBO removes direct reuse of the held-out ' ...
    'beacon range, not all sensor dependence.'];

output_mat = fullfile(reference_dir, 'lobo_reference_data.mat');
save(output_mat, 't', 'beacon_pos', 'range_clean', 'vehicle_depth', ...
    'lbl_full', 'ref_full_new', 'solve_full', 'fusion_full', ...
    'lbl_lobo', 'ref_lobo', 'solve_lobo', 'fusion_lobo', ...
    'original_ref', 'replication', 'reference_summary', ...
    'metadata', '-v7.3');

plot_reference_trajectories(original_ref, ref_full_new, ref_lobo, ...
    beacon_pos, figure_dir);
plot_reference_differences(t, ref_full_new, ref_lobo, figure_dir);
plot_withheld_residuals(t, solve_lobo, figure_dir);

completion_file = fullfile(log_dir, 'REFERENCE_COMPLETED.txt');
fid = fopen(completion_file, 'w');
assert(fid >= 0, 'Cannot create completion marker.');
fprintf(fid, 'Completed: %s\n', datestr(now, 31));
fprintf(fid, 'Output MAT: %s\n', output_mat);
fprintf(fid, 'All-four reproduction RMSE: %.6f m\n', replication.RMSE_m);
fclose(fid);

fprintf('LOBO reference construction completed: %s\n', datestr(now, 31));
fprintf('Saved: %s\n', output_mat);

%% Local functions
function [lbl_llh, diag_out] = solve_depth_constrained_lbl( ...
        beacon_pos, ranges, vehicle_depth, t, included, min_valid)
    N = numel(t);
    n_beacons = numel(beacon_pos);
    Re = 6378137.0;
    beacons_llh = zeros(n_beacons,3);
    range_matrix = nan(N,n_beacons);
    for b = 1:n_beacons
        beacons_llh(b,:) = beacon_pos{b}(:)';
        rb = ranges{b}(:);
        assert(numel(rb) == N, 'Range length mismatch for beacon %d.', b);
        range_matrix(:,b) = medfilt1(rb, 11, 'omitnan', 'truncate');
    end

    % A fixed frame based only on surveyed beacon coordinates.  Using the
    % held-out beacon coordinate here changes only the coordinate origin and
    % does not use its range observation.
    lat0 = mean(beacons_llh(:,1));
    lon0 = mean(beacons_llh(:,2));
    beacons_enu = zeros(n_beacons,3);
    beacons_enu(:,1) = (beacons_llh(:,2)-lon0) * Re * cos(lat0);
    beacons_enu(:,2) = (beacons_llh(:,1)-lat0) * Re;
    beacons_enu(:,3) = beacons_llh(:,3);

    lbl_llh = nan(N,4);
    valid_fix = false(N,1);
    residual_rmse_m = nan(N,1);
    geometry_condition = nan(N,1);
    valid_beacon_count = zeros(N,1);
    x_guess = mean(beacons_enu(included,1:2),1);
    opts = optimoptions('lsqnonlin', 'Display', 'off', ...
        'StepTolerance', 1e-6, 'FunctionTolerance', 1e-8, ...
        'OptimalityTolerance', 1e-8, 'MaxIterations', 100);

    for k = 1:N
        rk_all = range_matrix(k,:);
        valid = false(1,n_beacons);
        valid(included) = isfinite(rk_all(included)) & ...
            rk_all(included) > 500 & rk_all(included) < 8000;
        ids = find(valid);
        valid_beacon_count(k) = numel(ids);
        if numel(ids) < min_valid
            continue;
        end

        zk = vehicle_depth(k);
        rk = rk_all(ids)';
        bxy = beacons_enu(ids,1:2);
        bz = beacons_enu(ids,3);
        res_fun = @(p) sqrt((bxy(:,1)-p(1)).^2 + ...
            (bxy(:,2)-p(2)).^2 + (bz-zk).^2) - rk;
        try
            [p_sol, resnorm, residual, exitflag] = ...
                lsqnonlin(res_fun, x_guess, [], [], opts);
        catch solve_error
            warning('LOBO:LBLFailure', ...
                'LBL solve failed at epoch %d: %s', k, solve_error.message);
            continue;
        end
        if exitflag <= 0 || any(~isfinite(p_sol))
            continue;
        end

        predicted = sqrt((p_sol(1)-bxy(:,1)).^2 + ...
            (p_sol(2)-bxy(:,2)).^2 + (zk-bz).^2);
        J = [(p_sol(1)-bxy(:,1))./predicted, ...
             (p_sol(2)-bxy(:,2))./predicted];
        normal_matrix = J' * J;
        geometry_condition(k) = cond(normal_matrix);
        residual_rmse_m(k) = sqrt(resnorm / numel(ids));
        if ~isempty(residual) && all(isfinite(residual))
            residual_rmse_m(k) = sqrt(mean(residual.^2));
        end
        lbl_llh(k,1) = lat0 + p_sol(2)/Re;
        lbl_llh(k,2) = lon0 + p_sol(1)/(Re*cos(lat0));
        lbl_llh(k,3) = zk;
        lbl_llh(k,4) = t(k);
        valid_fix(k) = true;
        x_guess = p_sol;
    end

    diag_out = struct();
    diag_out.included_beacons = included;
    diag_out.minimum_valid_beacons = min_valid;
    diag_out.valid_fix = valid_fix;
    diag_out.valid_beacon_count = valid_beacon_count;
    diag_out.residual_rmse_m = residual_rmse_m;
    diag_out.geometry_condition = geometry_condition;
    diag_out.local_origin_rad = [lat0, lon0];
end

function [avp_ref, diag_out] = fuse_lbl_octans_dvl( ...
        lbl_llh, octans, vxy, vehicle_depth, t)
    glvs;
    N = numel(t);
    valid_fix = all(isfinite(lbl_llh(:,1:2)),2);
    first_fix = find(valid_fix,1,'first');
    assert(~isempty(first_fix), 'No valid LBL fix is available for fusion.');

    Oweb = d2r(0.2);
    dx0 = [0.002; Oweb; 0.1/glv.Re; 0.1/glv.Re];
    x0 = 0.1 * dx0;
    vk = [0, Oweb, 0, 0];
    rk = [2/glv.Re, 2/glv.Re];
    kf = myekf('init', 0.5, x0, dx0, vk, rk);
    pos0 = [lbl_llh(first_fix,1:2), vehicle_depth(first_fix)]';
    dr = mydr('init', pos0, [0;0;0], 0.5);

    avp_ref = nan(N,10);
    x_record = nan(N,4);
    p_record = nan(N,4);
    used_update = false(N,1);
    for k = first_fix:N
        dr = mydr('update', dr, vehicle_depth(k), octans(k,3), vxy(k,1:2));
        kf = myekf('fk', kf, dr);
        kf = myekf('algo', kf, 'T');
        if valid_fix(k)
            kf.yk = dr.pos(1:2) - lbl_llh(k,1:2)';
            kf = myekf('hk', kf, dr, 'LBL');
            kf = myekf('algo', kf, 'M', 'EKF');
            used_update(k) = true;
        end
        corrected_pos = dr.pos;
        corrected_pos(1:2) = corrected_pos(1:2) - kf.xk(end-1:end);
        avp_ref(k,:) = [dr.att; dr.vn; corrected_pos; t(k)]';
        x_record(k,:) = kf.xk';
        p_record(k,:) = diag(kf.Pxk)';
    end

    diag_out = struct('valid_fix',valid_fix,'used_update',used_update, ...
        'state',x_record,'covariance_diagonal',p_record, ...
        'first_valid_epoch',first_fix);
end

function stats = horizontal_error_stats(estimate, reference)
    valid = all(isfinite(estimate(:,7:8)),2) & ...
        all(isfinite(reference(:,7:8)),2);
    if ~any(valid)
        stats = struct('N',0,'RMSE_m',NaN,'Max_m',NaN, ...
            'P95_m',NaN,'Mean_m',NaN,'Final_m',NaN);
        return;
    end
    lat_ref = reference(valid,7);
    dN = (estimate(valid,7)-lat_ref) * 6378137.0;
    dE = (estimate(valid,8)-reference(valid,8)) .* ...
        (6378137.0*cos(lat_ref));
    e = hypot(dE,dN);
    stats = struct('N',numel(e),'RMSE_m',sqrt(mean(e.^2)), ...
        'Max_m',max(e),'P95_m',percentile_local(e,95), ...
        'Mean_m',mean(e),'Final_m',e(end));
end

function [residual, stats] = withheld_range_residual(ref, measured, beacon)
    measured = measured(:);
    predicted = nan(size(measured));
    valid_ref = all(isfinite(ref(:,7:9)),2);
    if any(valid_ref)
        predicted(valid_ref) = RCompu(ref(valid_ref,7:9), beacon(:)');
    end
    residual = measured-predicted;
    valid = isfinite(residual);
    x = residual(valid);
    if isempty(x)
        stats = struct('N',0,'Mean_m',NaN,'Std_m',NaN, ...
            'RMSE_m',NaN,'P95Abs_m',NaN);
    else
        stats = struct('N',numel(x),'Mean_m',mean(x), ...
            'Std_m',std(x),'RMSE_m',sqrt(mean(x.^2)), ...
            'P95Abs_m',percentile_local(abs(x),95));
    end
end

function plot_reference_trajectories(original_ref, full_new, refs, beacons, out_dir)
    [E0,N0] = local_xy(original_ref(:,7:8), original_ref(1,7:8));
    fig = figure('Visible','off','Color','w','Position',[100 100 900 700]);
    plot(E0,N0,'k-','LineWidth',1.5,'DisplayName','Original fused reference'); hold on;
    [E,N] = local_xy(full_new(:,7:8), original_ref(1,7:8));
    plot(E,N,'--','Color',[0.35 0.35 0.35],'LineWidth',1.3, ...
        'DisplayName','Rebuilt four-beacon reference');
    colors = lines(4);
    for j = 1:4
        [E,N] = local_xy(refs{j}(:,7:8), original_ref(1,7:8));
        plot(E,N,'Color',colors(j,:),'LineWidth',1.0, ...
            'DisplayName',sprintf('Reference without B%d',j));
    end
    for j = 1:4
        [Eb,Nb] = local_xy(beacons{j}(1:2), original_ref(1,7:8));
        plot(Eb,Nb,'p','MarkerSize',11,'MarkerFaceColor',colors(j,:), ...
            'Color',colors(j,:),'HandleVisibility','off');
        text(Eb,Nb,sprintf(' B%d',j),'FontWeight','bold');
    end
    axis equal; grid on; xlabel('East (m)'); ylabel('North (m)');
    title('Leave-one-beacon-out reference trajectories');
    legend('Location','bestoutside');
    exportgraphics(fig,fullfile(out_dir,'lobo_reference_trajectories.png'),'Resolution',240);
    exportgraphics(fig,fullfile(out_dir,'lobo_reference_trajectories.pdf'),'ContentType','vector');
    close(fig);
end

function plot_reference_differences(t, full_ref, refs, out_dir)
    fig = figure('Visible','off','Color','w','Position',[100 100 1000 700]);
    tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for j = 1:4
        nexttile;
        e = horizontal_error_series(refs{j},full_ref);
        plot(t,e,'LineWidth',1.0); grid on;
        xlabel('Time (s)'); ylabel('Horizontal difference (m)');
        title(sprintf('Reference without B%d',j));
    end
    title(tl,'LOBO reference difference from rebuilt four-beacon reference');
    exportgraphics(fig,fullfile(out_dir,'lobo_reference_differences.png'),'Resolution',240);
    exportgraphics(fig,fullfile(out_dir,'lobo_reference_differences.pdf'),'ContentType','vector');
    close(fig);
end

function plot_withheld_residuals(t, solve_lobo, out_dir)
    fig = figure('Visible','off','Color','w','Position',[100 100 1000 700]);
    tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for j = 1:4
        nexttile;
        plot(t,solve_lobo{j}.held_out_range_residual_m,'LineWidth',0.9); grid on;
        xlabel('Time (s)'); ylabel('Measured - predicted slant range (m)');
        title(sprintf('Held-out B%d',j));
    end
    title(tl,'Held-out beacon range consistency');
    exportgraphics(fig,fullfile(out_dir,'lobo_withheld_range_residuals.png'),'Resolution',240);
    exportgraphics(fig,fullfile(out_dir,'lobo_withheld_range_residuals.pdf'),'ContentType','vector');
    close(fig);
end

function e = horizontal_error_series(estimate, reference)
    e = nan(size(estimate,1),1);
    valid = all(isfinite(estimate(:,7:8)),2) & all(isfinite(reference(:,7:8)),2);
    lat = reference(valid,7);
    dN = (estimate(valid,7)-lat)*6378137.0;
    dE = (estimate(valid,8)-reference(valid,8)).*(6378137.0*cos(lat));
    e(valid) = hypot(dE,dN);
end

function [E,N] = local_xy(pos, origin)
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

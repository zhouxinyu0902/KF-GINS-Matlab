%% Exper_Geometry_Observability_Real
% Quantitative geometry and finite-time local distinguishability analysis
% along the measured sea-trial trajectory.
%
% The vehicle trajectory, DVL/compass inputs, acoustic update schedule, state
% model, window length, and state normalization are held fixed.  The beacon
% position/history is changed case by case, so differences among B1--B4 and
% the measured moving beacon describe beacon-dependent observation geometry.
%
% This script is a trajectory-conditioned linearized diagnostic.  It is not
% presented as a proof of global nonlinear observability.

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
output_root = fullfile(repo_root, 'data', 'psins', ...
    'Geometry_Observability_Real');
mat_dir = fullfile(output_root, 'mat');
table_dir = fullfile(output_root, 'tables');
figure_dir = fullfile(output_root, 'figures');
log_dir = fullfile(output_root, 'logs');
ensure_folder(mat_dir);
ensure_folder(table_dir);
ensure_folder(figure_dir);
ensure_folder(log_dir);

log_file = fullfile(log_dir, 'Exper_Geometry_Observability_Real.log');
if exist(log_file, 'file'), delete(log_file); end
diary(log_file);
diary_cleanup = onCleanup(@() diary('off')); %#ok<NASGU>

cfg = struct();
cfg.navigation_dt_s = 0.5;
cfg.acoustic_interval_s = 8;
cfg.range_noise_std_m = 5;
cfg.window_seconds = [400, 800, 1200];
cfg.primary_window_seconds = 800;
cfg.radial_threshold_deg = 15; % Secondary descriptive diagnostic only.
cfg.state_labels = ["deltaK","c1","c2","deltaLat","deltaLon"];
cfg.parameter_indices = 1:3;
cfg.position_indices = 4:5;
cfg.state_scale = [0.002; d2r(5); d2r(5); 5/glv.Re; 5/glv.Re];
cfg.rank_relative_tolerance = 1e-6;

fprintf('Measured-trajectory geometry analysis started: %s\n', datestr(now,31));
fprintf('Input: %s\n', input_file);
fprintf('Windows: %s s; primary window: %g s.\n', ...
    mat2str(cfg.window_seconds), cfg.primary_window_seconds);

needed = {'avp_LBL_DR','LBL_out','BCN','compass','vxy', ...
    'LatLonDepTran'};
S = load(input_file, needed{:});
for i = 1:numel(needed)
    assert(isfield(S,needed{i}), 'Missing input variable: %s', needed{i});
end

t = S.LBL_out.t(:);
reference = S.avp_LBL_DR;
N = numel(t);
assert(size(reference,1)==N && size(S.compass,1)==N && ...
    size(S.vxy,1)==N, 'Measured inputs do not share a common time axis.');
dt = median(diff(t));
assert(abs(dt-cfg.navigation_dt_s)<1e-9, 'Unexpected navigation interval.');

update_idx = find(t>0 & abs(mod(t,cfg.acoustic_interval_s))<1e-8);
update_time = t(update_idx);
M = numel(update_idx);
fprintf('Acoustic geometry epochs: %d.\n', M);

% Construct fixed-beacon and moving-beacon histories on the navigation grid.
cases = repmat(struct(),5,1);
for j = 1:4
    cases(j).name = sprintf('B%d',j);
    cases(j).type = 'fixed';
    cases(j).position = repmat(S.BCN{j}(:)',N,1);
end
moving_time = S.LatLonDepTran(4,:)';
moving_sparse = [d2r(S.LatLonDepTran(1,:))', ...
                 d2r(S.LatLonDepTran(2,:))', ...
                 S.LatLonDepTran(3,:)'];
cases(5).name = 'Moving';
cases(5).type = 'moving';
cases(5).position = [ ...
    interp1(moving_time,moving_sparse(:,1),t,'linear','extrap'), ...
    interp1(moving_time,moving_sparse(:,2),t,'linear','extrap'), ...
    interp1(moving_time,moving_sparse(:,3),t,'linear','extrap')];

% Linearize the common 5-state process model along the measured inputs.
[Phi_step, velocity_en, earth_scale] = build_process_linearization( ...
    reference, S.compass, S.vxy, dt);
Phi_update = build_update_transitions(Phi_step, update_idx);

all_window_tables = cell(numel(cases),numel(cfg.window_seconds));
summary_rows = cell(numel(cases)*numel(cfg.window_seconds),1);
summary_count = 0;

for c = 1:numel(cases)
    fprintf('Analyzing %s geometry (%s beacon)...\n', ...
        cases(c).name, cases(c).type);
    geometry = calculate_geometry(reference, S.compass(:,3), ...
        cases(c).position, t, update_idx, cfg);
    H_update = build_range_jacobians(reference, cases(c).position, ...
        update_idx, earth_scale);

    cases(c).geometry = geometry;
    cases(c).H_update = H_update;
    for w = 1:numel(cfg.window_seconds)
        window_s = cfg.window_seconds(w);
        window_count = round(window_s/cfg.acoustic_interval_s)+1;
        metrics = calculate_window_metrics(H_update,Phi_update,geometry, ...
            window_count,cfg);
        metrics.Case = repmat(string(cases(c).name),height(metrics),1);
        metrics.BeaconType = repmat(string(cases(c).type),height(metrics),1);
        metrics.Window_s = repmat(window_s,height(metrics),1);
        metrics = movevars(metrics,{'Case','BeaconType','Window_s'},'Before',1);
        all_window_tables{c,w} = metrics;

        summary_count = summary_count+1;
        summary_rows{summary_count} = summarize_metrics( ...
            cases(c), metrics, window_s, cfg);
    end
end

windowed_metrics = vertcat(all_window_tables{:});
summary_table = struct2table(vertcat(summary_rows{:}));
primary_summary = summary_table( ...
    summary_table.Window_s==cfg.primary_window_seconds,:);
primary_windowed = windowed_metrics( ...
    windowed_metrics.Window_s==cfg.primary_window_seconds,:);

% Attach LOBO navigation statistics when available.  They are descriptive
% context only and are not used to calculate the geometry metrics.
lobo_csv = fullfile(repo_root,'data','psins','LOBO','tables', ...
    'lobo_navigation_5state_summary.csv');
if exist(lobo_csv,'file')
    lobo = readtable(lobo_csv);
    primary_summary.LOBO_DR_RMSE_m = nan(height(primary_summary),1);
    primary_summary.LOBO_EKF_RMSE_m = nan(height(primary_summary),1);
    primary_summary.LOBO_Gain_Percent = nan(height(primary_summary),1);
    for j = 1:4
        row = primary_summary.Case==string(sprintf('B%d',j));
        source_row = lobo.HeldOutBeacon==j;
        primary_summary.LOBO_DR_RMSE_m(row) = lobo.DR_RMSE_LOBO_m(source_row);
        primary_summary.LOBO_EKF_RMSE_m(row) = lobo.EKF_RMSE_LOBO_m(source_row);
        primary_summary.LOBO_Gain_Percent(row) = ...
            lobo.EKF_GainVsDR_Percent(source_row);
    end
end

writetable(windowed_metrics,fullfile(table_dir, ...
    'geometry_observability_windowed_all.csv'));
writetable(primary_windowed,fullfile(table_dir, ...
    'geometry_observability_windowed_primary.csv'));
writetable(summary_table,fullfile(table_dir, ...
    'geometry_observability_window_sensitivity.csv'));
writetable(primary_summary,fullfile(table_dir, ...
    'geometry_observability_primary_summary.csv'));
%%
plot_geometry_layout(reference,cases,figure_dir);
plot_relative_bearing(cases,update_time,figure_dir);
plot_window_geometry(primary_windowed,figure_dir);
plot_observability(primary_windowed,figure_dir);
plot_primary_summary(primary_summary,figure_dir);

method_note = struct();
method_note.scope = ['Finite-time, trajectory-conditioned linearized ' ...
    'distinguishability diagnostic for the measured 5-state model.'];
method_note.control = ['The vehicle trajectory, sensor inputs, state model, ' ...
    'state scaling, update times, and window are fixed; only beacon geometry changes.'];
method_note.parameter_metric = ['The normalized parameter columns ' ...
    '[deltaK,c1,c2] are projected onto the orthogonal complement of the ' ...
    'two normalized position columns before singular values and coherence are computed.'];
method_note.limit = ['The metrics do not prove global nonlinear observability ' ...
    'and do not include process noise, Kalman gain, or closed-loop covariance updates.'];

save(fullfile(mat_dir,'geometry_observability_real.mat'), ...
    'cfg','t','update_idx','update_time','reference','cases', ...
    'Phi_step','Phi_update','velocity_en','earth_scale', ...
    'windowed_metrics','summary_table','primary_summary','method_note','-v7.3');

completion_file = fullfile(log_dir,'COMPLETED.txt');
fid = fopen(completion_file,'w');
assert(fid>=0,'Cannot create completion marker.');
fprintf(fid,'Completed: %s\n',datestr(now,31));
fprintf(fid,'Primary window: %.0f s\n',cfg.primary_window_seconds);
fprintf(fid,'Cases: B1, B2, B3, B4, Moving\n');
fclose(fid);

disp(primary_summary);
fprintf('Measured-trajectory geometry analysis completed: %s\n',datestr(now,31));

%% Local functions
function [Phi_step,velocity_en,earth_scale] = ...
        build_process_linearization(reference,compass,vxy,dt)
    N = size(reference,1);
    Phi_step = repmat(eye(5),1,1,N);
    velocity_en = nan(N,2);
    earth_scale = nan(N,4); % RMh, clRNh, sin(lat), cos(lat)
    for k = 1:N
        pos = reference(k,7:9)';
        Cnb = a2mat([0,0,compass(k,3)]);
        vn = Cnb*[vxy(k,1:2)';0];
        eth = earth(pos,vn);
        VE = vn(1); VN = vn(2); psi = compass(k,3);
        Ft = zeros(5,5);
        Ft(4,1) = VN/eth.RMh;
        Ft(4,2) = VE*cos(2*psi)/eth.RMh;
        Ft(4,3) = VE*sin(2*psi)/eth.RMh;
        Ft(5,1) = VE/eth.clRNh;
        Ft(5,2) = -VN*cos(2*psi)/eth.clRNh;
        Ft(5,3) = -VN*sin(2*psi)/eth.clRNh;
        Ft(5,4) = VE*eth.sl/(eth.clRNh*eth.cl);
        Phi_step(:,:,k) = eye(5)+Ft*dt;
        velocity_en(k,:) = vn(1:2)';
        earth_scale(k,:) = [eth.RMh,eth.clRNh,eth.sl,eth.cl];
    end
end

function Phi_update = build_update_transitions(Phi_step,update_idx)
    M = numel(update_idx);
    n = size(Phi_step,1);
    Phi_update = repmat(eye(n),1,1,M);
    for m = 2:M
        T = eye(n);
        for k = update_idx(m-1)+1:update_idx(m)
            T = Phi_step(:,:,k)*T;
        end
        Phi_update(:,:,m) = T;
    end
end

function H = build_range_jacobians(reference,beacon_history,idx,earth_scale)
    M = numel(idx);
    H = nan(M,5);
    for m = 1:M
        k = idx(m);
        dlat = reference(k,7)-beacon_history(k,1);
        dlon = reference(k,8)-beacon_history(k,2);
        RMh = earth_scale(k,1);
        clRNh = earth_scale(k,2);
        horizontal_range = hypot(dlat*RMh,dlon*clRNh);
        if horizontal_range<=0 || ~isfinite(horizontal_range)
            continue;
        end
        b_lat = dlat*RMh^2/horizontal_range;
        b_lon = dlon*clRNh^2/horizontal_range;
        H(m,:) = [0,0,0,b_lat,b_lon];
    end
end

function geometry = calculate_geometry(reference,heading,beacon_history,t,idx,cfg)
    origin = reference(1,7:8);
    [Ev,Nv] = local_xy(reference(:,7:8),origin);
    [Eb,Nb] = local_xy(beacon_history(:,1:2),origin);
    % Match the heading convention used by compareBeaconBearing.m and the
    % manuscript plots: azimuth = atan2(-East, North).
    beta = atan2(-(Eb-Ev),Nb-Nv);
    alpha = wrap_angle(beta-heading(:));
    relative_range = hypot(Eb-Ev,Nb-Nv);

    beta_u = beta(idx);
    alpha_u = alpha(idx);
    delta_beta = wrap_angle(diff(beta_u));
    los_rate = [NaN;abs(delta_beta)/cfg.acoustic_interval_s];
    radial = abs(alpha_u)<=d2r(cfg.radial_threshold_deg) | ...
        abs(alpha_u)>=d2r(180-cfg.radial_threshold_deg);

    geometry = struct();
    geometry.beta_rad = beta;
    geometry.relative_bearing_rad = alpha;
    geometry.horizontal_range_m = relative_range;
    geometry.beta_update_rad = beta_u;
    geometry.relative_bearing_update_rad = alpha_u;
    geometry.horizontal_range_m_at_update = relative_range(idx);
    geometry.delta_beta_update_rad = delta_beta;
    geometry.los_rate_update_rad_s = los_rate;
    geometry.radial_update = radial;
    geometry.update_time = t(idx);
end

function T = calculate_window_metrics(H,Phi_update,geometry,window_count,cfg)
    M = size(H,1);
    n_rows = max(0,M-window_count+1);
    Time_s = nan(n_rows,1);
    WindowStart_s = nan(n_rows,1);
    HorizontalRangeMedian_m = nan(n_rows,1);
    LOSVariation_deg = nan(n_rows,1);
    MedianLOSRate_deg_s = nan(n_rows,1);
    RadialFraction = nan(n_rows,1);
    FullSigmaRatio = nan(n_rows,1);
    FullLog10Condition = nan(n_rows,1);
    FullNumericalRank = nan(n_rows,1);
    ParameterSigmaRatio = nan(n_rows,1);
    ParameterLog10Condition = nan(n_rows,1);
    ParameterNumericalRank = nan(n_rows,1);
    ParameterMaxCoherence = nan(n_rows,1);
    D = diag(cfg.state_scale);

    row = 0;
    for m_end = window_count:M
        row = row+1;
        m_start = m_end-window_count+1;
        O = nan(window_count,5);
        Tprop = eye(5);
        O(1,:) = H(m_start,:);
        for q = m_start+1:m_end
            Tprop = Phi_update(:,:,q)*Tprop;
            O(q-m_start+1,:) = H(q,:)*Tprop;
        end
        valid_rows = all(isfinite(O),2);
        On = (O(valid_rows,:)/cfg.range_noise_std_m)*D;

        [eta_full,log_condition_full,rank_full] = ...
            singular_metrics(On,cfg.rank_relative_tolerance);
        Otheta = On(:,cfg.parameter_indices);
        Oposition = On(:,cfg.position_indices);
        Qposition = orth(Oposition);
        if isempty(Qposition)
            Otheta_residual = Otheta;
        else
            Otheta_residual = Otheta-Qposition*(Qposition'*Otheta);
        end
        [eta_parameter,log_condition_parameter,rank_parameter] = ...
            singular_metrics(Otheta_residual,cfg.rank_relative_tolerance);
        coherence = maximum_column_coherence(Otheta_residual);

        Time_s(row) = geometry.update_time(m_end);
        WindowStart_s(row) = geometry.update_time(m_start);
        HorizontalRangeMedian_m(row) = median( ...
            geometry.horizontal_range_m_at_update(m_start:m_end),'omitnan');
        db = geometry.delta_beta_update_rad(m_start:m_end-1);
        LOSVariation_deg(row) = r2d(sum(abs(db),'omitnan'));
        rates = geometry.los_rate_update_rad_s(m_start:m_end);
        MedianLOSRate_deg_s(row) = r2d(median(rates,'omitnan'));
        RadialFraction(row) = mean(geometry.radial_update(m_start:m_end));
        FullSigmaRatio(row) = eta_full;
        FullLog10Condition(row) = log_condition_full;
        FullNumericalRank(row) = rank_full;
        ParameterSigmaRatio(row) = eta_parameter;
        ParameterLog10Condition(row) = log_condition_parameter;
        ParameterNumericalRank(row) = rank_parameter;
        ParameterMaxCoherence(row) = coherence;
    end

    T = table(Time_s,WindowStart_s,HorizontalRangeMedian_m, ...
        LOSVariation_deg,MedianLOSRate_deg_s,RadialFraction, ...
        FullSigmaRatio,FullLog10Condition,FullNumericalRank, ...
        ParameterSigmaRatio,ParameterLog10Condition, ...
        ParameterNumericalRank,ParameterMaxCoherence);
end

function out = summarize_metrics(case_data,T,window_s,cfg)
    out = struct();
    out.Case = string(case_data.name);
    out.BeaconType = string(case_data.type);
    out.Window_s = window_s;
    out.WindowCount = height(T);
    out.MedianHorizontalRange_m = median(T.HorizontalRangeMedian_m,'omitnan');
    out.MedianLOSVariation_deg = median(T.LOSVariation_deg,'omitnan');
    out.P05LOSVariation_deg = percentile_local(T.LOSVariation_deg,5);
    out.MedianLOSRate_deg_s = median(T.MedianLOSRate_deg_s,'omitnan');
    out.MedianRadialFraction = median(T.RadialFraction,'omitnan');
    out.MedianFullSigmaRatio = median(T.FullSigmaRatio,'omitnan');
    out.P05FullSigmaRatio = percentile_local(T.FullSigmaRatio,5);
    out.MedianFullLog10Condition = median(T.FullLog10Condition,'omitnan');
    out.MinimumFullRank = min(T.FullNumericalRank,[],'omitnan');
    out.MedianParameterSigmaRatio = median(T.ParameterSigmaRatio,'omitnan');
    out.P05ParameterSigmaRatio = percentile_local(T.ParameterSigmaRatio,5);
    out.MedianParameterLog10Condition = ...
        median(T.ParameterLog10Condition,'omitnan');
    out.MinimumParameterRank = min(T.ParameterNumericalRank,[],'omitnan');
    out.MedianParameterMaxCoherence = ...
        median(T.ParameterMaxCoherence,'omitnan');
    out.P95ParameterMaxCoherence = ...
        percentile_local(T.ParameterMaxCoherence,95);
    out.RadialThreshold_deg = cfg.radial_threshold_deg;
end

function [ratio,log_condition,numerical_rank] = singular_metrics(A,rel_tol)
    if isempty(A) || any(~isfinite(A),'all')
        ratio = NaN; log_condition = NaN; numerical_rank = NaN; return;
    end
    s = svd(A,'econ');
    if isempty(s) || s(1)<=0
        ratio = 0; log_condition = Inf; numerical_rank = 0; return;
    end
    numerical_rank = sum(s>s(1)*rel_tol);
    if numel(s)<size(A,2)
        ratio = 0;
    else
        ratio = s(end)/s(1);
    end
    if ratio>0
        log_condition = -log10(ratio);
    else
        log_condition = Inf;
    end
end

function coherence = maximum_column_coherence(A)
    n = size(A,2);
    C = nan(n,n);
    for i = 1:n
        for j = i+1:n
            ni = norm(A(:,i)); nj = norm(A(:,j));
            if ni>0 && nj>0
                C(i,j) = abs(A(:,i)'*A(:,j))/(ni*nj);
            end
        end
    end
    values = C(isfinite(C));
    if isempty(values), coherence = NaN; else, coherence = max(values); end
end

function plot_geometry_layout(reference,cases,out_dir)
    origin = reference(1,7:8);
    [Ev,Nv] = local_xy(reference(:,7:8),origin);
    fig = figure('Visible','off','Color','w','Position',[100 100 900 700]);
    plot(Ev,Nv,'k-','LineWidth',1.5,'DisplayName','Measured vehicle trajectory'); hold on;
    colors = lines(numel(cases));
    for c = 1:numel(cases)
        [Eb,Nb] = local_xy(cases(c).position(:,1:2),origin);
        if strcmp(cases(c).type,'fixed')
            plot(Eb(1),Nb(1),'p','MarkerSize',12, ...
                'MarkerFaceColor',colors(c,:),'Color',colors(c,:), ...
                'DisplayName',cases(c).name);
            text(Eb(1),Nb(1),sprintf(' %s',cases(c).name),'FontWeight','bold');
        else
            plot(Eb,Nb,'Color',colors(c,:),'LineWidth',1.2, ...
                'DisplayName','Measured moving beacon');
        end
    end
    axis equal; grid on; xlabel('East (m)'); ylabel('North (m)');
    title('Measured trajectory and beacon geometries');
    legend('Location','bestoutside');
    export_pair(fig,out_dir,'geometry_layout'); close(fig);
end

function plot_relative_bearing(cases,update_time,out_dir)
    fig = figure('Visible','off','Color','w','Position',[100 100 1000 650]);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    colors = lines(numel(cases));
    nexttile; hold on;
    for c = 1:numel(cases)
        plot(update_time,r2d(cases(c).geometry.relative_bearing_update_rad), ...
            'Color',colors(c,:),'LineWidth',0.9,'DisplayName',cases(c).name);
    end
    yline(0,':'); yline(180,':'); yline(-180,':'); grid on;
    ylabel('Heading-relative bearing (deg)'); legend('Location','bestoutside');
    nexttile; hold on;
    for c = 1:numel(cases)
        plot(update_time,r2d(cases(c).geometry.los_rate_update_rad_s), ...
            'Color',colors(c,:),'LineWidth',0.9,'DisplayName',cases(c).name);
    end
    grid on; xlabel('Time (s)'); ylabel('|LOS angular rate| (deg/s)');
    title('Instantaneous LOS rotation');
    export_pair(fig,out_dir,'geometry_relative_bearing_and_los_rate'); close(fig);
end

function plot_window_geometry(T,out_dir)
    cases = unique(T.Case,'stable');
    colors = lines(numel(cases));
    fig = figure('Visible','off','Color','w','Position',[100 100 1000 650]);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    nexttile; hold on;
    for c = 1:numel(cases)
        q = T.Case==cases(c);
        plot(T.Time_s(q),T.LOSVariation_deg(q),'Color',colors(c,:), ...
            'LineWidth',1.0,'DisplayName',cases(c));
    end
    grid on; ylabel('Window LOS variation (deg)'); legend('Location','bestoutside');
    nexttile; hold on;
    for c = 1:numel(cases)
        q = T.Case==cases(c);
        plot(T.Time_s(q),T.RadialFraction(q),'Color',colors(c,:), ...
            'LineWidth',1.0,'DisplayName',cases(c));
    end
    grid on; xlabel('Window end time (s)'); ylabel('Near-radial fraction');
    export_pair(fig,out_dir,'geometry_window_metrics'); close(fig);
end

function plot_observability(T,out_dir)
    cases = unique(T.Case,'stable');
    colors = lines(numel(cases));
    % fig = figure('Visible','off','Color','w','Position',[100 100 1000 650]);
    fig = myfigurestartup(7,3,'paper');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    nexttile; hold on;
    for c = 1:numel(cases)
        q = T.Case==cases(c);
        semilogy(T.Time_s(q),max(T.FullSigmaRatio(q),realmin), ...
            'Color',colors(c,:),'LineWidth',1.0,'DisplayName',cases(c));
    end
    grid on; ylabel('Full-state singular-value ratio'); legend('Location','bestoutside');
    nexttile; hold on;
    for c = 1:numel(cases)
        q = T.Case==cases(c);
        semilogy(T.Time_s(q),max(T.ParameterSigmaRatio(q),realmin), ...
            'Color',colors(c,:),'LineWidth',1.0,'DisplayName',cases(c));
    end
    grid on; xlabel('Window end time (s)');
    ylabel('Parameter singular-value ratio');
    export_pair(fig,out_dir,'observability_singular_value_ratios'); close(fig);
end

function plot_primary_summary(T,out_dir)
    fig = figure('Visible','off','Color','w','Position',[100 100 1000 480]);
    tiledlayout(1,3,'TileSpacing','compact','Padding','compact');
    nexttile; bar(T.MedianLOSVariation_deg); grid on;
    xticklabels(T.Case); ylabel('Median LOS variation (deg)');
    nexttile;
    full_values = max(T.MedianFullSigmaRatio,realmin);
    bar(full_values); set(gca,'YScale','log');
    ylim([min(full_values)/2,max(full_values)*2]); grid on;
    xticklabels(T.Case); ylabel('Median full-state ratio');
    nexttile;
    parameter_values = max(T.MedianParameterSigmaRatio,realmin);
    bar(parameter_values); set(gca,'YScale','log');
    ylim([min(parameter_values)/2,max(parameter_values)*2]); grid on;
    xticklabels(T.Case); ylabel('Median parameter ratio');
    export_pair(fig,out_dir,'geometry_observability_summary'); close(fig);
end

function export_pair(fig,out_dir,name)
    exportgraphics(fig,fullfile(out_dir,[name,'.png']),'Resolution',240);
    exportgraphics(fig,fullfile(out_dir,[name,'.pdf']),'ContentType','vector');
end

function [E,N] = local_xy(pos,origin)
    E = (pos(:,2)-origin(2))*6378137.0*cos(origin(1));
    N = (pos(:,1)-origin(1))*6378137.0;
end

function y = wrap_angle(x)
    y = atan2(sin(x),cos(x));
end

function value = percentile_local(x,p)
    x = sort(x(isfinite(x)));
    if isempty(x), value = NaN; return; end
    q = 1+(numel(x)-1)*p/100;
    lo = floor(q); hi = ceil(q);
    if lo==hi
        value = x(lo);
    else
        value = x(lo)+(q-lo)*(x(hi)-x(lo));
    end
end

function ensure_folder(path_value)
    if ~exist(path_value,'dir'), mkdir(path_value); end
end

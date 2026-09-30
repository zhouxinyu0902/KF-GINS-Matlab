%% Reviewer 4-4 / 4-5: parameter error, 3-sigma bounds and state coupling
% Run Exper1_Dr_Range_new.m with exper = 0 and state = 5 first.

clear; clc; close all;

data_file = 'D:\Github\KF-GINS-Matlab\data\psins\datasaved_new\data_simu_5state_reviewer_4_4_4_5.mat';
if ~exist(data_file,'file')
    data_file = 'D:\Github\KF-GINS-Matlab\data\psins\datasaved_new\data_simu_5state.mat';
end
output_dir = fullfile(fileparts(data_file), 'reviewer_4_4_4_5');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

S = load(data_file);
required_fields = {'XK','PkFull','beacon_data','parameter_truth_5state','vk'};
for k = 1:numel(required_fields)
    assert(isfield(S, required_fields{k}), ...
        'Missing variable "%s". Re-run the modified Exper1_Dr_Range_new.m.', ...
        required_fields{k});
end

% The first case following all fixed beacons is the primary moving beacon B9.
case_id = size(S.beacon_data.fixed_pos, 1) + 1;
assert(case_id <= numel(S.XK) && case_id <= numel(S.PkFull), ...
    'Moving-beacon case B9 is not available in the saved result.');

X = S.XK{case_id};
P_record = S.PkFull{case_id};
assert(size(X,2) == 6, 'The selected result is not a five-state result.');
assert(size(P_record,2) == 26, ...
    'PkFull must contain 25 covariance elements followed by time.');

n = min(size(X,1), size(P_record,1));
X = X(1:n,:);
P_record = P_record(1:n,:);
time_s = X(:,end);
time_min = (time_s-time_s(1))/60;

truth = S.parameter_truth_5state(:);
estimate = X(:,1:3);
error_raw = estimate-truth.';

sigma_raw = zeros(n,3);
for k = 1:n
    Pk = reshape(P_record(k,1:25),5,5);
    Pk = (Pk+Pk.')/2;
    sigma_raw(k,:) = sqrt(max(diag(Pk(1:3,1:3)),0)).';
end

% Display deltaK in percent and c1/c2 in degrees.
scale = [100, 180/pi, 180/pi];
error_plot = error_raw.*scale;
sigma_plot = sigma_raw.*scale;
labels = {'\delta K','c_1','c_2'};
units = {'%','deg','deg'};

% fig = figure('Color','w','Position',[100 100 1120 760]);
fig = myfigurestartup(7,4,'paper');
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');

for j = 1:3
    ax = nexttile(tl,j);
    plot(ax,time_min,error_plot(:,j),'k-','LineWidth',1.25); hold(ax,'on');
    plot(ax,time_min, 3*sigma_plot(:,j),'r--','LineWidth',1.05);
    plot(ax,time_min,-3*sigma_plot(:,j),'r--','LineWidth',1.05);
    yline(ax,0,'Color',[0.45 0.45 0.45],'LineStyle',':');
    grid(ax,'on'); box(ax,'on');
    xlabel(ax,'Time (min)');
    ylabel(ax,sprintf('%s error (%s)',labels{j},units{j}), ...
        'Interpreter','tex');
    if j == 1
        legend(ax,{'Estimation error','+3\sigma','-3\sigma'}, ...
            'Interpreter','tex','Location','best');
    end
end

% Normalize the final posterior covariance to show dimensionless coupling.
P_final = reshape(P_record(n,1:25),5,5);
P_final = (P_final+P_final.')/2;
std_final = sqrt(max(diag(P_final),0));
den = std_final*std_final.';
rho = zeros(5);
valid = den > 0;
rho(valid) = P_final(valid)./den(valid);
rho(1:6:end) = 1;
rho = max(min(rho,1),-1);

ax = nexttile(tl,4);
imagesc(ax,rho,[-1 1]);
axis(ax,'image'); box(ax,'on');
colormap(ax,blue_white_red(257));
cb = colorbar(ax);
cb.Label.String = 'Correlation coefficient';
state_labels = {'\delta K','c_1','c_2','\delta L','\delta \lambda'};
set(ax,'XTick',1:5,'XTickLabel',state_labels, ...
    'YTick',1:5,'YTickLabel',state_labels,'TickLabelInterpreter','tex');
xlabel(ax,'State'); ylabel(ax,'State');
title(ax,'Final posterior state correlation','FontWeight','normal');
for row = 1:5
    for col = 1:5
        if abs(rho(row,col)) > 0.55
            text_color = 'w';
        else
            text_color = 'k';
        end
        text(ax,col,row,sprintf('%.2f',rho(row,col)), ...
            'HorizontalAlignment','center','Color',text_color,'fontsize',8);
    end
end

title(tl,'Five-state moving-beacon simulation (B9)','FontName','TimesSimSun','fontsize',10);

exportgraphics(fig,fullfile(output_dir,'Reviewer_4_4_4_5_parameter_consistency.png'), ...
    'Resolution',600);
exportgraphics(fig,fullfile(output_dir,'Reviewer_4_4_4_5_parameter_consistency.pdf'), ...
    'ContentType','vector');

rmse_value = sqrt(mean(error_plot.^2,1,'omitnan')).';
final_error = error_plot(end,:).';
coverage_pct = 100*mean(abs(error_plot) <= 3*sigma_plot,1,'omitnan').';
stats = table(string(labels(:)),string(units(:)),rmse_value,final_error,coverage_pct, ...
    'VariableNames',{'Parameter','Unit','RMSE','FinalError','CoverageWithin3Sigma_pct'});
writetable(stats,fullfile(output_dir,'Reviewer_4_4_4_5_parameter_statistics.csv'));

fprintf('Five-state continuous process-noise amplitudes v:\n');
fprintf('  deltaK = %.6g 1/sqrt(s)\n',S.vk(1));
fprintf('  c1     = %.6g deg/sqrt(s)\n',rad2deg(S.vk(2)));
fprintf('  c2     = %.6g deg/sqrt(s)\n',rad2deg(S.vk(3)));
disp(stats);

function cmap = blue_white_red(n)
if nargin < 1
    n = 257;
end
n1 = ceil(n/2);
n2 = n-n1+1;
blue = [0.10 0.30 0.80];
white = [1.00 1.00 1.00];
red = [0.80 0.15 0.10];
left = [linspace(blue(1),white(1),n1).', ...
        linspace(blue(2),white(2),n1).', ...
        linspace(blue(3),white(3),n1).'];
right = [linspace(white(1),red(1),n2).', ...
         linspace(white(2),red(2),n2).', ...
         linspace(white(3),red(3),n2).'];
cmap = [left; right(2:end,:)];
end

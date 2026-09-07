function [cleaned_data, is_anomaly, valid_indices] = clean_outliers_dynamic_knn(data, time_vec, threshold, K, do_plot, y_label_str)
    % --- 1. Default parameter settings ---
    if nargin < 6 || isempty(y_label_str); y_label_str = 'Value'; end
    if nargin < 5 || isempty(do_plot); do_plot = true; end
    if nargin < 4 || isempty(K); K = 10; end
    if nargin < 3 || isempty(threshold); threshold = 80; end
    if nargin < 2 || isempty(time_vec); time_vec = 1:length(data); end

    % Ensure row vectors
    data = data(:).';
    time_vec = time_vec(:).';

    N_total = length(data);
    is_anomaly = false(1, N_total);

    % Initialize the first finite sample as valid
    first_valid = find(isfinite(data) & isfinite(time_vec), 1, 'first');

    if isempty(first_valid)
        warning('No finite samples are available. Returning the original data.');
        cleaned_data = data;
        valid_indices = [];
        is_anomaly(:) = true;
        return;
    end

    valid_indices = first_valid;

    % Mark non-finite samples as anomalies
    is_anomaly(~isfinite(data) | ~isfinite(time_vec)) = true;
    is_anomaly(first_valid) = false;

    % --- 2. Dynamic KNN-based outlier detection ---
    for i = first_valid+1:N_total

        if ~isfinite(data(i)) || ~isfinite(time_vec(i))
            is_anomaly(i) = true;
            continue;
        end

        num_ref = min(K, length(valid_indices));
        ref_idx = valid_indices(end - num_ref + 1 : end);
        ref_values = data(ref_idx);

        distances = abs(data(i) - ref_values);

        if mean(distances, 'omitnan') > threshold
            is_anomaly(i) = true;
        else
            valid_indices(end+1) = i;
        end
    end

    % --- 3. Data reconstruction ---
    raw_data = data;
    cleaned_data = data;

    if any(is_anomaly)

        t_valid = time_vec(~is_anomaly);
        v_valid = data(~is_anomaly);
        t_target = time_vec(is_anomaly);

        idx_valid = isfinite(t_valid) & isfinite(v_valid);
        t_valid = t_valid(idx_valid);
        v_valid = v_valid(idx_valid);

        [t_valid, unique_idx] = unique(t_valid, 'stable');
        v_valid = v_valid(unique_idx);

        if numel(t_valid) >= 2
            v_interp = interp1(t_valid, v_valid, t_target, 'pchip', 'extrap');
            cleaned_data(is_anomaly) = v_interp;
        elseif numel(t_valid) == 1
            warning('Only one valid sample remains. Constant filling is used.');
            cleaned_data(is_anomaly) = v_valid(1);
        else
            warning('No valid sample remains. The original data are returned.');
            cleaned_data = raw_data;
        end

        fprintf('--- Dynamic KNN Outlier Detection and Reconstruction Report ---\n');
        fprintf('Detected and reconstructed outliers: %d\n', sum(is_anomaly));
    end

    % --- 4. Visualization ---
    if do_plot

    fig = myfigurestartup(3, 3, 'zxy');
    set(fig, 'Name', 'Dynamic KNN Outlier Detection and Reconstruction');

    % Color settings for journal-style figures
    color_raw   = [0.75 0.75 0.75];   % light gray
    color_rec   = [0.120 0.330 0.600]; % softened deep blue
    color_out   = [0.720 0.180 0.220]; % softened wine red
    color_fill  = [0.902 0.624 0.000]; % muted orange

    plot(time_vec, raw_data, '--', ...
        'Color', color_raw, ...
        'LineWidth', 1.0, ...
        'DisplayName', 'Raw data'); 
    hold on;

    plot(time_vec, cleaned_data, '-', ...
        'Color', color_rec, ...
        'LineWidth', 1.6, ...
        'DisplayName', 'Reconstructed data');

    plot(time_vec(is_anomaly), raw_data(is_anomaly), 'x', ...
        'Color', color_out, ...
        'LineWidth', 1.2, ...
        'MarkerSize', 7, ...
        'DisplayName', 'Detected outliers');

    plot(time_vec(is_anomaly), cleaned_data(is_anomaly), 'o', ...
        'Color', color_fill, ...
        'LineWidth', 1.1, ...
        'MarkerSize', 5.5, ...
        'MarkerFaceColor', 'none', ...
        'DisplayName', 'Reconstructed samples');

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel(y_label_str, 'Interpreter', 'none');

    legend('Location', 'best');

    xlim([time_vec(1), time_vec(end)]);

end
end
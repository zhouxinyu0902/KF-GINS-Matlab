function results = compare_height_vertical_rmse(truthFile, navFiles, varargin)
%COMPARE_HEIGHT_VERTICAL_RMSE Compare height and vertical velocity RMSE.
%
% results = compare_height_vertical_rmse(truthFile, navFiles)
% results = compare_height_vertical_rmse(..., 'MethodNames', names)
% results = compare_height_vertical_rmse(..., 'OutputFile', csvPath)
% results = compare_height_vertical_rmse(..., 'FigureFile', figPath)
%
% File convention:
%   truth/nav column 2: time, s
%   truth/nav column 5: height, m
%   truth/nav column 8: vertical velocity vD, m/s
%
% The truth trajectory is interpolated onto each estimated nav time axis.
% RMSE is computed only on the overlapping time interval.

    parser = inputParser;
    parser.addRequired('truthFile', @(x) ischar(x) || isstring(x));
    parser.addRequired('navFiles', @(x) ischar(x) || isstring(x) || iscell(x));
    parser.addParameter('MethodNames', [], @(x) isstring(x) || iscellstr(x) || ischar(x));
    parser.addParameter('OutputFile', "", @(x) ischar(x) || isstring(x));
    parser.addParameter('FigureFile', "", @(x) ischar(x) || isstring(x));
    parser.addParameter('Plot', true, @(x) islogical(x) && isscalar(x));
    parser.addParameter('TimeRange', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 2));
    parser.addParameter('TimeColumnTruth', 2, @(x) isnumeric(x) && isscalar(x));
    parser.addParameter('TimeColumnNav', 2, @(x) isnumeric(x) && isscalar(x));
    parser.addParameter('HeightColumn', 5, @(x) isnumeric(x) && isscalar(x));
    parser.addParameter('VerticalVelocityColumn', 8, @(x) isnumeric(x) && isscalar(x));
    parser.parse(truthFile, navFiles, varargin{:});

    opts = parser.Results;
    truthFile = string(opts.truthFile);
    navFiles = normalizeFileList(opts.navFiles);
    methodNames = normalizeMethodNames(opts.MethodNames, navFiles);

    truth = loadNumericMatrix(truthFile, "truth");
    requiredTruthCols = max([opts.TimeColumnTruth, opts.HeightColumn, opts.VerticalVelocityColumn]);
    if size(truth, 2) < requiredTruthCols
        error("truth file requires at least %d columns: %s", requiredTruthCols, truthFile);
    end

    truthTime = truth(:, opts.TimeColumnTruth);
    truthHeight = truth(:, opts.HeightColumn);
    truthVd = truth(:, opts.VerticalVelocityColumn);
    [truthTime, uniqueIdx] = unique(truthTime, "stable");
    truthHeight = truthHeight(uniqueIdx);
    truthVd = truthVd(uniqueIdx);

    if numel(truthTime) < 2
        error("truth file must contain at least two unique time epochs.");
    end

    n = numel(navFiles);
    heightRmse = nan(n, 1);
    verticalVelocityRmse = nan(n, 1);
    startTime = nan(n, 1);
    endTime = nan(n, 1);
    sampleCount = zeros(n, 1);
    maxAbsHeightError = nan(n, 1);
    maxAbsVerticalVelocityError = nan(n, 1);

    errSeries = cell(n, 1);

    for k = 1:n
        nav = loadNumericMatrix(navFiles(k), "navigation result");
        requiredNavCols = max([opts.TimeColumnNav, opts.HeightColumn, opts.VerticalVelocityColumn]);
        if size(nav, 2) < requiredNavCols
            error("nav file requires at least %d columns: %s", requiredNavCols, navFiles(k));
        end

        navTime = nav(:, opts.TimeColumnNav);
        navHeight = nav(:, opts.HeightColumn);
        navVd = nav(:, opts.VerticalVelocityColumn);
        [navTime, uniqueIdx] = unique(navTime, "stable");
        navHeight = navHeight(uniqueIdx);
        navVd = navVd(uniqueIdx);

        validTime = navTime >= truthTime(1) & navTime <= truthTime(end);
        if ~isempty(opts.TimeRange)
            validTime = validTime & navTime >= opts.TimeRange(1) & navTime <= opts.TimeRange(2);
        end

        navTime = navTime(validTime);
        navHeight = navHeight(validTime);
        navVd = navVd(validTime);

        if isempty(navTime)
            warning("No overlapping samples for file: %s", navFiles(k));
            continue;
        end

        refHeight = interp1(truthTime, truthHeight, navTime, "linear");
        refVd = interp1(truthTime, truthVd, navTime, "linear");

        heightError = navHeight - refHeight;
        vdError = navVd - refVd;

        heightRmse(k) = rmseFinite(heightError);
        verticalVelocityRmse(k) = rmseFinite(vdError);
        maxAbsHeightError(k) = max(abs(heightError(isfinite(heightError))));
        maxAbsVerticalVelocityError(k) = max(abs(vdError(isfinite(vdError))));
        startTime(k) = navTime(1);
        endTime(k) = navTime(end);
        sampleCount(k) = numel(navTime);

        errSeries{k} = struct( ...
            'time', navTime, ...
            'heightError', heightError, ...
            'verticalVelocityError', vdError);
    end

    results = table(methodNames(:), navFiles(:), startTime, endTime, sampleCount, ...
        heightRmse, verticalVelocityRmse, maxAbsHeightError, maxAbsVerticalVelocityError, ...
        'VariableNames', {'Method', 'File', 'StartTime', 'EndTime', 'Samples', ...
        'HeightRMSE_m', 'VerticalVelocityRMSE_mps', ...
        'MaxAbsHeightError_m', 'MaxAbsVerticalVelocityError_mps'});

    disp(results);

    outputFile = string(opts.OutputFile);
    if strlength(outputFile) > 0
        outputFolder = fileparts(outputFile);
        if strlength(string(outputFolder)) > 0 && ~isfolder(outputFolder)
            mkdir(outputFolder);
        end
        writetable(results, outputFile);
        fprintf("RMSE results written to: %s\n", outputFile);
    end

    if opts.Plot
        plotErrors(errSeries, methodNames);

        figureFile = string(opts.FigureFile);
        if strlength(figureFile) > 0
            figureFolder = fileparts(figureFile);
            if strlength(string(figureFolder)) > 0 && ~isfolder(figureFolder)
                mkdir(figureFolder);
            end
            exportgraphics(gcf, figureFile, 'Resolution', 300);
            fprintf("Error figure written to: %s\n", figureFile);
        end
    end
end

function files = normalizeFileList(filesIn)
    if iscell(filesIn)
        files = string(filesIn);
    else
        files = string(filesIn);
    end
    files = files(:);
    if isempty(files)
        error("navFiles must contain at least one file.");
    end
end

function names = normalizeMethodNames(namesIn, files)
    if isempty(namesIn)
        names = strings(numel(files), 1);
        for k = 1:numel(files)
            [~, baseName, ext] = fileparts(files(k));
            names(k) = baseName + ext;
        end
        return;
    end

    names = string(namesIn);
    names = names(:);
    if numel(names) ~= numel(files)
        error("MethodNames must have the same length as navFiles.");
    end
end

function data = loadNumericMatrix(filePath, dataName)
    if ~isfile(filePath)
        error("%s file does not exist: %s", dataName, filePath);
    end

    data = importdata(filePath);
    if isstruct(data)
        data = data.data;
    end

    if ~isnumeric(data) || isempty(data)
        error("%s file contains no numeric data: %s", dataName, filePath);
    end
    if any(~isfinite(data(:)))
        error("%s file contains NaN or Inf values: %s", dataName, filePath);
    end
end

function value = rmseFinite(errorVector)
    valid = isfinite(errorVector);
    if ~any(valid)
        value = nan;
        return;
    end
    errorVector = errorVector(valid);
    value = sqrt(mean(errorVector.^2));
end

function plotErrors(errSeries, methodNames)
    % figure('Color', 'w', 'Name', 'Height and Vertical Velocity Error');
    myfigurestartup(7,5,'zxy');
    
    % 为了安全获取全局的时间起点和终点
    t_start = 0; t_end = 100; % 默认 fallback
    for k = 1:numel(errSeries)
        if ~isempty(errSeries{k})
            time = errSeries{k}.time-errSeries{k}.time(1);
            t_start = time(1);
            t_end = time(end);
            break;
        end
    end
    
    %% ======================= 子图 1：高度误差 =======================
    ax1 = subplot(2, 1, 1);
    hold on;
    grid on;
    for k = 1:numel(errSeries)
        if isempty(errSeries{k})
            continue;
        end
        time = errSeries{k}.time-errSeries{k}.time(1);
        plot(time, errSeries{k}.heightError, 'LineWidth', 1.0);
    end
    xlabel('Time (s)');
    ylabel('Height error (m)');
    title('Height Error');
    legend(methodNames, 'Interpreter', 'none', 'Location', 'best');
    xlim([t_start t_end]);
    
    % --------- 局部放大（高度） ---------
    ax_inset1 = axes('Position', [0.6 0.65 0.25 0.18]); 
    hold on; 
    grid on;
    box on; 
    
    for k = 1:numel(errSeries)
        if isempty(errSeries{k})
            continue;
        end
        time = errSeries{k}.time-errSeries{k}.time(1);
        plot(time, errSeries{k}.heightError, 'LineWidth', 1.0);
    end
    
    xlim(ax_inset1, [t_start, t_start + 50]);
    ylim(ax_inset1, [-1, 1]);
    ax_inset1.FontSize = 8;
    title(ax_inset1, 'Zoom In', 'FontSize', 9, 'FontWeight', 'normal');
    
    %% ======================= 子图 2：垂向速度误差 =======================
    ax2 = subplot(2, 1, 2);
    hold on;
    grid on;
    for k = 1:numel(errSeries)
        if isempty(errSeries{k})
            continue;
        end
        time = errSeries{k}.time-errSeries{k}.time(1);
        plot(time, errSeries{k}.verticalVelocityError, 'LineWidth', 1.0);
    end
    xlabel('Time (s)');
    ylabel('Vertical velocity error (m/s)');
    title('Vertical Velocity Error');
    legend(methodNames, 'Interpreter', 'none', 'Location', 'best');
    xlim([t_start t_end]);
    
    % --------- 局部放大（垂向速度） ---------
    % Position 的第二个参数(底部距离)从 0.65 改为 0.22，使其落在下方子图的右上角
    ax_inset2 = axes('Position', [0.6 0.22 0.25 0.18]); 
    hold on; 
    grid on;
    box on; 
    
    for k = 1:numel(errSeries)
        if isempty(errSeries{k})
            continue;
        end
        time = errSeries{k}.time-errSeries{k}.time(1);
        plot(time, errSeries{k}.verticalVelocityError, 'LineWidth', 1.0);
    end
    
    xlim(ax_inset2, [1800, 2600]);
    % 这里的速度误差范围我预设为 ±0.5，你可以根据实际曲线的收敛情况调整
    ylim(ax_inset2, [-1, 0.5]); 
    
    ax_inset2.FontSize = 8;
    title(ax_inset2, 'Zoom In', 'FontSize', 9, 'FontWeight', 'normal');
end
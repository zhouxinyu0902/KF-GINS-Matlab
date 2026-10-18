%% 整体分析USBL的数据
clear;
clc;
close all;

%% Paths and USBL parsing
scriptsDir = 'D:\Github\KF-GINS-Matlab\20230504海试数据\标定试验';
projectDir = 'D:\Github\KF-GINS-Matlab\data\20230504\标定实验分析';
dataDir = fullfile(projectDir, "信标标定试验数据");
outputDir = fullfile(projectDir, "解析后的数据", "usbl_analysis");
parserFile = fullfile(scriptsDir, "parse_usbl.m");

mustBeFolder(dataDir);
mustBeFile(parserFile);
if ~isfolder(outputDir), mkdir(outputDir); end
addpath(scriptsDir);
cleanupPath = onCleanup(@() rmpath(scriptsDir)); 

parserScratch = string(tempname);
mkdir(parserScratch);
cleanupScratch = onCleanup(@() cleanup_parser_scratch(parserScratch)); 

displayUtcOffsetHours = 8;
usblSpec = fullfile(dataDir, "USBL-BOX-R-*.log");
usbl = parse_usbl(usblSpec, parserScratch);

%% Parsed USBL tables
write_txt(usbl.ptsag, fullfile(outputDir, "usbl_ptsag.txt"));
write_txt(usbl.ptsah, fullfile(outputDir, "usbl_ptsah.txt"));
write_txt(usbl.echo_utc, fullfile(outputDir, "usbl_echo_utc.txt"));
write_txt(usbl.message_counts, fullfile(outputDir, "usbl_message_counts.txt"));

%% Overall summaries by file batch and transponder ID
ptsag = usbl.ptsag;
ptsah = usbl.ptsah;
echoUtc = usbl.echo_utc;
sourceFiles = unique(string(ptsag.source_file), 'stable');
nBatches = numel(sourceFiles);

batchIndex = (1:nBatches)';
fileName = strings(nBatches,1);
filenameClock = NaT(nBatches,1,'TimeZone','UTC');
ptsagCount = zeros(nBatches,1);
ptsahCount = zeros(nBatches,1);
echoUtcCount = zeros(nBatches,1);
rawUtcStart = NaT(nBatches,1,'TimeZone','UTC');
rawUtcEnd = NaT(nBatches,1,'TimeZone','UTC');
displayClockStart = NaT(nBatches,1,'TimeZone','UTC');
displayClockEnd = NaT(nBatches,1,'TimeZone','UTC');
durationMin = nan(nBatches,1);
filenameMinusDisplayStartMin = nan(nBatches,1);
transponderIds = strings(nBatches,1);
checksumOkPct = nan(nBatches,1);
idRows = cell(0,1);

for b = 1:nBatches
    src = sourceFiles(b);
    [~,stem,ext] = fileparts(char(src));
    fileName(b) = string([stem,ext]);
    filenameClock(b) = filename_clock_from_name(fileName(b));
    q = ptsag(string(ptsag.source_file) == src,:);
    ptsagCount(b) = height(q);
    ptsahCount(b) = sum(string(ptsah.source_file) == src);
    echoUtcCount(b) = sum(string(echoUtc.source_file) == src);
    rawUtcStart(b) = min(q.utc_time);
    rawUtcEnd(b) = max(q.utc_time);
    displayClockStart(b) = rawUtcStart(b) + hours(displayUtcOffsetHours);
    displayClockEnd(b) = rawUtcEnd(b) + hours(displayUtcOffsetHours);
    durationMin(b) = minutes(rawUtcEnd(b)-rawUtcStart(b));
    if ~isnat(filenameClock(b))
        filenameMinusDisplayStartMin(b) = minutes( ...
            filenameClock(b)-displayClockStart(b));
    end
    idsInBatch = unique(q.transponder_id,'stable');
    transponderIds(b) = strjoin(string(idsInBatch'),',');
    checksumOkPct(b) = 100*mean(q.checksum_ok);

    for j = 1:numel(idsInBatch)
        id = idsInBatch(j);
        z = q(q.transponder_id == id,:);
        flags = unique(string(z.position_flag),'stable');
        flagParts = strings(numel(flags),1);
        for f = 1:numel(flags)
            flagParts(f) = flags(f) + ":" + ...
                sum(string(z.position_flag) == flags(f));
        end
        depth = z.computed_depth_m(isfinite(z.computed_depth_m));
        if isempty(depth)
            depthStats = [NaN,NaN,NaN];
        else
            depthStats = [min(depth),median(depth),max(depth)];
        end
        idRows{end+1,1} = table(b,fileName(b),id,height(z), ... %#ok<AGROW>
            min(z.utc_time),max(z.utc_time), ...
            finite_min(z.latitude_deg),finite_max(z.latitude_deg), ...
            finite_min(z.longitude_deg),finite_max(z.longitude_deg), ...
            depthStats(1),depthStats(2),depthStats(3), ...
            strjoin(flagParts,','),100*mean(z.checksum_ok), ...
            'VariableNames', {'batch_index','file_name','transponder_id', ...
            'ptsag_count','raw_utc_start','raw_utc_end','latitude_min_deg', ...
            'latitude_max_deg','longitude_min_deg','longitude_max_deg', ...
            'computed_depth_min_m','computed_depth_median_m', ...
            'computed_depth_max_m','position_flag_counts','checksum_ok_pct'});
    end
end

batchSummary = table(batchIndex,fileName,filenameClock,ptsagCount, ...
    ptsahCount,echoUtcCount,rawUtcStart,rawUtcEnd,displayClockStart, ...
    displayClockEnd,durationMin,filenameMinusDisplayStartMin, ...
    transponderIds,checksumOkPct, ...
    'VariableNames', {'batch_index','file_name','filename_clock', ...
    'ptsag_count','ptsah_count','echo_utc_count','raw_utc_start', ...
    'raw_utc_end','display_clock_start','display_clock_end','duration_min', ...
    'filename_minus_display_start_min','transponder_ids','checksum_ok_pct'});
idSummary = vertcat(idRows{:});

messageType = ["PTSAG";"PTSAH";"ECHO UTC"];
rowCount = [height(ptsag);height(ptsah);height(echoUtc)];
parsedContents = [ ...
    "frame, UTC, transponder ID, latitude, longitude, position flag, computed/device depth"; ...
    "device ID, ship heading, auxiliary value"; ...
    "received UTC, echoed UTC"];
contentSummary = table(messageType,rowCount,parsedContents, ...
    'VariableNames', {'message_type','row_count','parsed_contents'});

write_txt(batchSummary,fullfile(outputDir,"usbl_batch_summary.txt"));
write_txt(idSummary,fullfile(outputDir,"usbl_id_summary.txt"));
write_txt(contentSummary,fullfile(outputDir,"usbl_content_summary.txt"));

%% USBL-only four-panel overview figure
ids = unique(ptsag.transponder_id,'stable');
colors = lines(numel(ids));
lineStyles = {'-','--',':','-.'};
pointOnlyIds = [91,93];
% fig = figure('Color','w','Visible','off','Position',[60,60,1500,900]);
fig = myfigurestartup(7,7,'paper');
layout = tiledlayout(fig,2,2,'Padding','compact','TileSpacing','compact');

ax1 = nexttile(layout,1);
hold(ax1,'on');
for j = 1:numel(ids)
    if ismember(ids(j),pointOnlyIds)
        mask = ptsag.transponder_id == ids(j) & ...
            isfinite(ptsag.latitude_deg) & isfinite(ptsag.longitude_deg);
        scatter(ax1,ptsag.longitude_deg(mask),ptsag.latitude_deg(mask), ...
            12,colors(j,:),'filled','DisplayName',"ID "+ids(j));
        continue;
    end
    firstSegment = true;
    for b = 1:nBatches
        mask = ptsag.transponder_id == ids(j) & ...
            string(ptsag.source_file) == sourceFiles(b) & ...
            isfinite(ptsag.latitude_deg) & isfinite(ptsag.longitude_deg);
        if ~any(mask), continue; end
        if firstSegment
            label = "ID "+ids(j); visibility = 'on'; firstSegment = false;
        else
            label = ""; visibility = 'off';
        end
        h = plot(ax1,ptsag.longitude_deg(mask),ptsag.latitude_deg(mask), ...
            'Color',colors(j,:), ...
            'LineStyle',lineStyles{mod(b-1,numel(lineStyles))+1}, ...
            'LineWidth',1.1,'DisplayName',label);
        h.HandleVisibility = visibility;
    end
end
grid(ax1,'on'); axis(ax1,'equal');
xlabel(ax1,'Longitude (deg)'); ylabel(ax1,'Latitude (deg)');
title(ax1,'(a) PTSAG positions'); legend(ax1,'Location','best');

ax2 = nexttile(layout,2);
hold(ax2,'on');
for b = 1:nBatches
    plot(ax2,[displayClockStart(b),displayClockEnd(b)],[b,b],'-', ...
        'Color',[0.75,0.75,0.75],'LineWidth',5,'HandleVisibility','off');
end
for j = 1:numel(ids)
    legendShown = false;
    for b = 1:nBatches
        mask = ptsag.transponder_id == ids(j) & ...
            string(ptsag.source_file) == sourceFiles(b);
        idx = decimate_indices(find(mask),700);
        if isempty(idx), continue; end
        if legendShown
            label = ""; visibility = 'off';
        else
            label = "ID "+ids(j); visibility = 'on'; legendShown = true;
        end
        h = scatter(ax2,ptsag.utc_time(idx)+hours(displayUtcOffsetHours), ...
            repmat(b+0.09*(j-(numel(ids)+1)/2),numel(idx),1), ...
            9,colors(j,:),'filled','DisplayName',label);
        h.HandleVisibility = visibility;
    end
end
grid(ax2,'on'); yticks(ax2,1:nBatches); yticklabels(ax2,"B"+(1:nBatches));
xlabel(ax2,sprintf('Message UTC %+g h (display clock)',displayUtcOffsetHours));
ylabel(ax2,'Source file batch'); title(ax2,'(b) Batch coverage and ID availability');
xtickformat(ax2,'MM-dd HH:mm');

ax3 = nexttile(layout,3);
hold(ax3,'on');
for j = 1:numel(ids)
    mask = ptsag.transponder_id == ids(j) & ...
        isfinite(ptsag.computed_depth_m) & ptsag.computed_depth_m > 0;
    idx = decimate_indices(find(mask),2500);
    scatter(ax3,ptsag.utc_time(idx)+hours(displayUtcOffsetHours), ...
        ptsag.computed_depth_m(idx),8,colors(j,:),'filled', ...
        'DisplayName',"ID "+ids(j));
end
set(ax3,'YScale','log'); grid(ax3,'on');
xlabel(ax3,sprintf('Message UTC %+g h (display clock)',displayUtcOffsetHours));
ylabel(ax3,'Computed depth (m, log scale)'); title(ax3,'(c) PTSAG computed depth');
legend(ax3,'Location','best'); xtickformat(ax3,'MM-dd HH:mm');

ax4 = nexttile(layout,4);
bar(ax4,[ptsagCount,ptsahCount,echoUtcCount],'grouped'); grid(ax4,'on');
xticks(ax4,1:nBatches); xticklabels(ax4,"B"+(1:nBatches));
xlabel(ax4,'Source file batch'); ylabel(ax4,'Rows');
title(ax4,'(d) Parsed message counts');
legend(ax4,{'PTSAG','PTSAH','ECHO UTC'},'Location','best');
for b = 1:nBatches
    y = max([ptsagCount(b),ptsahCount(b),echoUtcCount(b)]);
    text(ax4,b,y*1.04,"IDs "+transponderIds(b), ...
        'HorizontalAlignment','center','FontSize',9);
end
ylim(ax4,[0,1.15*max([ptsagCount;ptsahCount;echoUtcCount])]);

title(layout,sprintf(['USBL-only overview | %d files, %d PTSAG rows | ' ...
    'UTC %+g h \n shown only as display clock'], ...
    nBatches,height(ptsag),displayUtcOffsetHours), ...
    'FontWeight','bold','FontSize',14);

pngFile = fullfile(outputDir,"usbl_overview.png");
pdfFile = fullfile(outputDir,"usbl_overview.pdf");
exportgraphics(fig,pngFile,'Resolution',220);
exportgraphics(fig,pdfFile,'ContentType','vector');
close(fig);

save(fullfile(outputDir,"usbl_analysis.mat"), ...
    'usbl','batchSummary','idSummary','contentSummary','-v7.3');

fprintf('\nUSBL overall analysis completed.\n');
fprintf('  Files: %d\n',nBatches);
fprintf('  PTSAG: %d; PTSAH: %d; ECHO UTC: %d\n', ...
    height(ptsag),height(ptsah),height(echoUtc));
fprintf('  Figure: %s\n',pngFile);

%% Local helpers
function write_txt(tbl,file)
writetable(tbl,file,'FileType','text','Delimiter','\t','Encoding','UTF-8');
end

function cleanup_parser_scratch(folder)
folder = string(folder);
tempRoot = string(tempdir);
if isfolder(folder) && startsWith(lower(folder),lower(tempRoot))
    rmdir(folder,'s');
end
end

function value = finite_min(x)
x = x(isfinite(x));
if isempty(x), value = NaN; else, value = min(x); end
end

function value = finite_max(x)
x = x(isfinite(x));
if isempty(x), value = NaN; else, value = max(x); end
end

function dt = filename_clock_from_name(fileName)
token = regexp(char(fileName),'(20\d{6})-(\d{6})','tokens','once');
if isempty(token)
    dt = NaT(1,1,'TimeZone','UTC');
else
    dt = datetime([token{1},token{2}], ...
        'InputFormat','yyyyMMddHHmmss','TimeZone','UTC');
end
end

function idx = decimate_indices(idx,maxPoints)
if numel(idx) > maxPoints
    idx = idx(round(linspace(1,numel(idx),maxPoints)));
end
end

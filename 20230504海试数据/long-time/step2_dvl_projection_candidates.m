%% 第二步辅助诊断：尝试多种 DVL 坐标和姿态角约定
% 使用 step2 已生成的原始 DVL、nav 和 IMU 数据，比较不同投影假设。

clear; clc; close all;
scriptDir = string(fileparts(mfilename('fullpath')));
repoDir = fileparts(fileparts(scriptDir));
stage2Dir = fullfile(repoDir,"data","20230504","实际海试","case01","processed","stage2");
matFile = fullfile(stage2Dir,'step2_case01_0017.mat');
if ~isfile(matFile), error('PHINS:MissingStage2','请先运行 step2_prepare_case01_0017.m。'); end
S = load(matFile,'dvl','nav','imuRaw');

figureHelperDir = fullfile(repoDir,"psins2401","mytest","00_all_func");
addpath(figureHelperDir);
cleanupFigurePath = onCleanup(@() rmpath(figureHelperDir)); %#ok<NASGU>

dvl = S.dvl; nav = S.nav; t0 = S.imuRaw.utc_time(1);
dvlTimeS = posixtime(dvl.utc_time);
navTimeS = posixtime(nav.utc_time);
heading = interp_field(navTimeS,nav.heading_deg,dvlTimeS);
roll = interp_field(navTimeS,nav.roll_deg,dvlTimeS);
pitch = interp_field(navTimeS,nav.pitch_deg,dvlTimeS);
insN = interp_field(navTimeS,nav.speedNorth_mps,dvlTimeS);
insE = interp_field(navTimeS,nav.speedEast_mps,dvlTimeS);
insD = -interp_field(navTimeS,nav.speedUp_mps,dvlTimeS);
raw = [dvl.speedX_mps,dvl.speedY_mps,dvl.speedZ_mps];

mappingNames = ["PHINS_FLU";"RDI_TO_PHINS_FLU";"RDI_ALT";"RAW_FRD";"RFU_TO_FRD";"FLU_TO_FRD"];
mappingMatrices = cat(3, ...
    [1 0 0;0 1 0;0 0 1], ...
    [0 1 0;-1 0 0;0 0 1], ...
    [0 -1 0;1 0 0;0 0 1], ...
    [1 0 0;0 -1 0;0 0 -1], ...
    [0 1 0;1 0 0;0 0 -1], ...
    [1 0 0;0 -1 0;0 0 -1]);

rows = cell(0,12);
for im = 1:numel(mappingNames)
    body0 = (mappingMatrices(:,:,im) * raw.').';
    for headingSign = [-1,1]
        for rollSign = [-1,1]
            for pitchSign = [-1,1]
                for orderId = 1:2
                    if orderId == 1, orderName = "RxRy"; else, orderName = "RyRx"; end
                    objective = @(mountYawDeg) candidate_cost(body0,heading,roll,pitch, ...
                        insN,insE,insD,headingSign,rollSign,pitchSign,orderId,mountYawDeg);
                    mountYawDeg = fminbnd(objective,-180,180,optimset('TolX',1e-3));
                    [vN,vE,vD] = project_candidate(body0,heading,roll,pitch, ...
                        headingSign,rollSign,pitchSign,orderId,mountYawDeg);
                    stats = candidate_stats(vN,vE,vD,insN,insE,insD);
                    rows(end+1,:) = {mappingNames(im),headingSign,rollSign,pitchSign,orderName, ...
                        mountYawDeg,stats.horizontalRmsTrimmed,stats.threeDimRmsTrimmed, ...
                        stats.northRms,stats.eastRms,stats.downRms,stats.horizontalCorrelation}; %#ok<AGROW>
                end
            end
        end
    end
end

candidateTable = cell2table(rows,'VariableNames',{'mapping','heading_sign','roll_sign', ...
    'pitch_sign','rotation_order','mount_yaw_deg','horizontal_rms_trimmed_mps', ...
    'three_dim_rms_trimmed_mps','north_rms_mps','east_rms_mps','down_rms_mps', ...
    'horizontal_correlation'});
candidateTable = sortrows(candidateTable,{'horizontal_rms_trimmed_mps','three_dim_rms_trimmed_mps'});
writetable(candidateTable,fullfile(stage2Dir,'log1_DVL投影候选方案统计.txt'), ...
    'FileType','text','Delimiter','\t','Encoding','UTF-8');

best = candidateTable(1,:);
bestBody = (mappingMatrices(:,:,find(mappingNames==best.mapping)) * raw.').';
[bestN,bestE,bestD] = project_candidate(bestBody,heading,roll,pitch, ...
    best.heading_sign,best.roll_sign,best.pitch_sign,find(["RxRy","RyRx"]==best.rotation_order),best.mount_yaw_deg);
bestComparison = table(dvl.utc_time,seconds(dvl.utc_time-t0),bestN,bestE,bestD,insN,insE,insD, ...
    bestN-insN,bestE-insE,bestD-insD, ...
    'VariableNames',{'utc_time','time_s','dvl_north_mps','dvl_east_mps','dvl_down_mps', ...
    'ins_north_mps','ins_east_mps','ins_down_mps','north_error_mps','east_error_mps','down_error_mps'});
writetable(bestComparison,fullfile(stage2Dir,'log1_DVL最佳候选方案对比.txt'), ...
    'FileType','text','Delimiter','\t','Encoding','UTF-8');

fig = myfigurestartup(7,7,'paper');
set(fig,'Visible','off'); delete(findall(fig,'Type','axes'));
tiledlayout(fig,3,1,'Padding','compact','TileSpacing','compact');
fieldsD = ["dvl_north_mps";"dvl_east_mps";"dvl_down_mps"];
fieldsI = ["ins_north_mps";"ins_east_mps";"ins_down_mps"];
names = ["北向";"东向";"下向"];
for k = 1:3
    ax = nexttile; hold(ax,'on');
    plot(ax,bestComparison.utc_time,bestComparison.(fieldsD(k)),'DisplayName','DVL 最佳候选');
    plot(ax,bestComparison.utc_time,bestComparison.(fieldsI(k)),'--','DisplayName','INS 导航');
    grid(ax,'on'); xlabel(ax,'时间'); ylabel(ax,'速度 / m·s^{-1}');
    title(ax,"候选最优方案（"+char('a'+k-1)+"）"+names(k)+"速度");
    legend(ax,'Location','best'); xtickformat(ax,'HH:mm');
end
exportgraphics(fig,fullfile(stage2Dir,'图6_DVL投影候选最佳方案.png'),'Resolution',300,'BackgroundColor','white');
close(fig);

fprintf('DVL 候选方案诊断完成。\n');
disp(best);

function value = interp_field(timeS,data,timeQueryS)
mask = isfinite(timeS) & isfinite(data);
[timeUnique,idx] = unique(timeS(mask),'stable');
dataUnique = data(mask); dataUnique = dataUnique(idx);
value = interp1(timeUnique,dataUnique,timeQueryS,'linear',NaN);
end

function cost = candidate_cost(body0,heading,roll,pitch,insN,insE,insD,hs,rs,ps,orderId,mountYawDeg)
[vN,vE,vD] = project_candidate(body0,heading,roll,pitch,hs,rs,ps,orderId,mountYawDeg);
err2 = (vN-insN).^2+(vE-insE).^2+(vD-insD).^2;
err2 = sort(err2(isfinite(err2)));
keep = max(2,floor(0.90*numel(err2)));
cost = sqrt(mean(err2(1:keep)));
end

function [vN,vE,vD] = project_candidate(body0,heading,roll,pitch,hs,rs,ps,orderId,mountYawDeg)
n = size(body0,1); vN=nan(n,1); vE=nan(n,1); vD=nan(n,1);
mount = deg2rad(mountYawDeg); cm=cos(mount); sm=sin(mount);
mountR = [cm,-sm,0;sm,cm,0;0,0,1];
c0 = diag([1,-1,-1]);
for k=1:n
    if any(~isfinite([heading(k),roll(k),pitch(k),body0(k,:)])), continue; end
    psi=deg2rad(hs*heading(k)); phi=deg2rad(rs*roll(k)); theta=deg2rad(ps*pitch(k));
    cp=cos(psi); sp=sin(psi); cf=cos(phi); sf=sin(phi); ct=cos(theta); st=sin(theta);
    rz=[cp,-sp,0;sp,cp,0;0,0,1];
    rx=[1,0,0;0,cf,-sf;0,sf,cf];
    ry=[ct,0,st;0,1,0;-st,0,ct];
    if orderId==1, rb=rx*ry; else, rb=ry*rx; end
    vn=rz*c0*rb*mountR*body0(k,:).';
    vN(k)=vn(1); vE(k)=vn(2); vD(k)=vn(3);
end
end

function stats = candidate_stats(vN,vE,vD,insN,insE,insD)
errN=vN-insN; errE=vE-insE; errD=vD-insD;
mask=isfinite(errN)&isfinite(errE)&isfinite(errD);
horizontal=sqrt(errN.^2+errE.^2); three=sqrt(errN.^2+errE.^2+errD.^2);
trim=mask; values=sort(horizontal(mask)); keep=max(2,floor(0.90*numel(values)));
stats.horizontalRmsTrimmed=sqrt(mean(values(1:keep).^2));
values=sort(three(mask)); keep=max(2,floor(0.90*numel(values)));
stats.threeDimRmsTrimmed=sqrt(mean(values(1:keep).^2));
stats.northRms=sqrt(mean(errN(mask).^2)); stats.eastRms=sqrt(mean(errE(mask).^2));
stats.downRms=sqrt(mean(errD(mask).^2));
speedD=sqrt(vN.^2+vE.^2); speedI=sqrt(insN.^2+insE.^2); m=isfinite(speedD)&isfinite(speedI);
stats.horizontalCorrelation=corr(speedD(m),speedI(m));
end

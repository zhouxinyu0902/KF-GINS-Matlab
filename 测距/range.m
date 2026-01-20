% Est_range = T2HorizR(DepthShipPIXOG,DepthHovCABIN,TimeH1PIXOG,SVP);
clear all
load("matlab1.mat");
load("matlab(2).mat");
treal=t1;
tideal=t;
fid=fopen('svp.txt','rt');
DistData=fscanf(fid,'%d.%d, %d.%d  %c\n',[5,inf]);
fclose(fid);
svp_ideal=read_svp_with_slash("svp.txt");
svp_real=importdata("svp_real.txt");

svp_Real.soundspeed=svp_real(:,2);
svp_Real.depth=svp_real(:,1);
figure;
plot(svp_ideal.soundspeed, -svp_ideal.depth);  % 深度为负值
hold on
plot(svp_Real.soundspeed, -svp_Real.depth)


xlabel('声速 (m/s)');
ylabel('深度 (m)');
title('声速剖面图');
grid on;
%%
for ii=1:50
    % time=(tideal(:,1)+randn*20)/1e6;

    % time=(treal(:,1)+randn*10)/1e6;
    % svp=svp_Real;

    time=(tideal(:,1)+randn*1000)/1e6;
    svp=svp_ideal;
    for i=1:10
        CZ = svp.depth';
        CV = svp.soundspeed'+randn*0.2;% 仪器0.02m/s精度
        SD = 500+rand*0.5; % 深度计记录的深度
        RD = 1000;% 母船的深度
        tt = time(i); % 传播时间
        % 根据声速剖面和深度计算积分
        CSD = interp1(CZ,CV,SD,'linear');  % linear, spline
        CRD = interp1(CZ,CV,RD,'linear');
        if SD < RD % 应答器在下
            ind = find( CZ>SD & CZ<RD );
            CZsr = [SD CZ(ind) RD];
            CVsr = [CSD CV(ind) CRD];
            Sa = trapz(CZsr,CVsr);
            gg = 2*Sa/(RD-SD).^2 - 2*CSD/abs(RD-SD);
            zs = SD;                       % source depth
            cs = CSD;                      % sound speed at source depth
            zr = RD;                        % receiver depth
            cr = cs + gg*(zr-zs);       % sound speed at receiver depth

            % Arithmetic mean speed
            cams = Sa/(RD-SD);

            % Geometric mean speed
            cgms = (RD-SD)/trapz(CZsr,1./CVsr);

            % zhou method
            n = 0;
            %         Ca = 0.00000001/max(CVsr);
            %         Cb = 0.99999999/max(CVsr);
            Ca = 0;
            Cb = min(1./CVsr);
            while 1
                fa = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Ca*CVsr).^2))) - tt;
                fb = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Cb*CVsr).^2))) - tt;
                C1 = (Ca+Cb)/2;
                f1 = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(C1*CVsr).^2))) - tt;
                if abs(f1) < 1e-5
                    break;
                else if fa*f1 < 0
                        Cb = C1;
                else if fb*f1 < 0
                        Ca = C1;
                end
                end
                end
                n = n+1;
                if n > 100
                    break; % travel time is small than depth
                end
            end
            CC = C1;
            hzhou = trapz(CZsr,CC*CVsr./sqrt(1-(CC*CVsr).^2));
        else
            ind = find( CZ<SD & CZ>RD );
            CZsr = [RD CZ(ind) SD];
            CVsr = [CRD CV(ind) CSD];
            Sa = trapz(CZsr,CVsr);
            gg = 2*CSD/(SD-RD) - 2*Sa/(SD-RD).^2;
            zs = SD;                       % source depth
            cs = CSD;                      % sound speed at source depth
            zr = RD;                        % receiver depth
            cr = cs + gg*(zr-zs);       % sound speed at receiver depth

            % Arithmetic mean speed
            cams = Sa/(SD-RD);

            % Geometric mean speed
            cgms = (SD-RD)/trapz(CZsr,1./CVsr);

            % zhou method
            n = 0;
            %         Ca = 0.00000001/max(CVsr);
            %         Cb = 0.99999999/max(CVsr);
            Ca = 0;
            Cb = min(1./CVsr);
            while 1
                fa = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Ca*CVsr).^2))) - tt;
                fb = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Cb*CVsr).^2))) - tt;
                C1 = (Ca+Cb)/2;
                f1 = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(C1*CVsr).^2))) - tt;
                if abs(f1) < 1e-5
                    break;
                else if fa*f1 < 0
                        Cb = C1;
                else if fb*f1 < 0
                        Ca = C1;
                end
                end
                end
                n = n+1;
                if n > 100
                    break; % travel time is small than depth
                end
            end
            CC = C1;
            hzhou = trapz(CZsr,CC*CVsr./sqrt(1-(CC*CVsr).^2));
        end

        hr = abs(1/gg) * sqrt( abs( (cs*exp(tt*gg)-cr)*(cs-cr*exp(tt*gg)) ) / exp(tt*gg) );  % horziontal range from source to receiver
        hams = sqrt( (tt*cams)^2 - abs(SD-RD)^2 );
        hgms = sqrt( (tt*cgms)^2 - abs(SD-RD)^2 );

        Est_range(1,i) = hr; %roundn(hr,-1);        % Equation
        Est_range(2,i) = hzhou; %roundn(hams,-1);   % zhou method
        Est_range(3,i) = hams; % roundn(hgms,-1);
        Est_range(4,i) = hgms; % roundn(hgms,-1);
    end
    error(:,ii)=Est_range(1,:)-t1(:,2)';
end
%%
figure
plot(t1(:,2)',Est_range(1,:)-t1(:,2)')
hold on
plot(t1(:,2)',Est_range(2,:)-t1(:,2)')
plot(t1(:,2)',Est_range(3,:)-t1(:,2)')
plot(t1(:,2)',Est_range(4,:)-t1(:,2)')
xlabel('距声源距离')
ylabel('测距误差')
legend('Equation','zhou method','hams','hgms')
%%
figure
plot(t1(:,2)',error(:,1),'.')
hold on
for i=2:50
    plot(t1(:,2)',error(:,i),'.')
end
xlabel('距声源距离')
ylabel('测距误差')
%%
figure
subplot 121
% 绘制箱型图
boxplot(error') %% 10*50的矩阵

% 添加标题和坐标轴标签
title('50次蒙特卡洛实验测距误差/m')
xlabel('距离声源的距离/km')
ylabel('测距误差')

% 如果需要设置组别的标签，可以使用以下方式
set(gca,'XTickLabel',round(t1(:,2)'/1000,2))
load("err.mat")

subplot 122
boxplot(err*1000) %% 10*50的矩阵

% 添加标题和坐标轴标签
title('50次蒙特卡洛实验测距误差/m')
xlabel('距离声源的距离/km')
ylabel('测距误差')
tt=linspace(16,20,10);
% 如果需要设置组别的标签，可以使用以下方式
set(gca,'XTickLabel',round(tt,2))
load("err.mat")
%%
function svp_data = read_svp_with_slash(filename)
% 读取带斜杠分隔符的SVP文件
% 格式：深度 声速 /

fid = fopen(filename, 'r');
if fid == -1
    error('无法打开文件: %s', filename);
end

% 方法1：忽略斜杠
data = [];
while ~feof(fid)
    line = fgetl(fid);
    if ~isempty(line)
        values = sscanf(line, '%f %f /');
        if length(values) >= 2
            data = [data; values(1), values(2)];
        end
    end
end
fclose(fid);

svp_data.depth = data(:,1);
svp_data.soundspeed = data(:,2);
end
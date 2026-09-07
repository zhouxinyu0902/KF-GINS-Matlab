function Est_range= PropaTcmp(DepthShipPIXOG,DepthHovCABIN,TimeH1PIXOG)
% 使用接收时间计算距离（导入声速剖面）
fid=fopen('SSP_1000.txt','rt'); % 声速剖面数据
SVP=fscanf(fid,'%f %f\n',[2,inf]);
CZ = SVP(1,:);
CV = SVP(2,:);

% Smooth SVP
CV_smooth = smooth(CZ,CV,0.001,'rloess').';% Adjust 0.02 to make the fit better

% figure; clf;
% plot( CV,CZ,'b.', CV_smooth,CZ,'r.'); axis('ij');  % real SSP
% xlim([1460 1560]); 
% xlabel('Sound Speed (m/s)'); ylabel('Depth (m)');
% legend('Actual','Filtered','Location','NorthEast');

CV = CV_smooth;
LengthPIXOGRaw=length(DepthShipPIXOG);
numintergral = zeros(1,LengthPIXOGRaw);
Est_range = zeros(4,LengthPIXOGRaw);
for ii = 1 : LengthPIXOGRaw 
    SD = DepthHovCABIN(ii); % 深度计记录的深度
    RD = DepthShipPIXOG(ii);% 母船的深度
    t = TimeH1PIXOG(ii); % 传播时间
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
            fa = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Ca*CVsr).^2))) - t;
            fb = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Cb*CVsr).^2))) - t;        
            C1 = (Ca+Cb)/2;
            f1 = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(C1*CVsr).^2))) - t;
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
        numintergral(1,ii) = n;
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
            fa = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Ca*CVsr).^2))) - t;
            fb = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(Cb*CVsr).^2))) - t;        
            C1 = (Ca+Cb)/2;
            f1 = trapz(CZsr,(1./CVsr).*(1./sqrt(1-(C1*CVsr).^2))) - t;
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
        numintergral(1,ii) = n;
    end
    
    hr = abs(1/gg) * sqrt( abs( (cs*exp(t*gg)-cr)*(cs-cr*exp(t*gg)) ) / exp(t*gg) );  % horziontal range from source to receiver
    hams = sqrt( (t*cams)^2 - abs(SD-RD)^2 );  
    hgms = sqrt( (t*cgms)^2 - abs(SD-RD)^2 );  
    % if ((t*cams)^2 - abs(SD-RD)^2)<0  keyboard; end
    % if ((t*cgms)^2 - abs(SD-RD)^2)<0  keyboard; end
    if (abs( (cs*exp(t*gg)-cr)*(cs-cr*exp(t*gg)) ) / exp(t*gg))<0 keyboard;end
    Est_range(1,ii) = hr; %roundn(hr,-1);        % Equation
    Est_range(2,ii) = hzhou; %roundn(hams,-1);   % zhou method
    Est_range(3,ii) = hams; % roundn(hgms,-1);
    Est_range(4,ii) = hgms; % roundn(hgms,-1);

    
%     Est_range(5,ii) = RR*1000;
end
end


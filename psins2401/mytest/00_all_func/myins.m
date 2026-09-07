function ins = myins(type,varargin)
glvs
switch(type)
    case 'initial'
        [ts,avp0]=setvals(varargin); % 变量赋值
        ins = [];
        ins.ts = ts; ins.nts = 2*ts;
        [qnb0, vn0, pos0] = setvals(a2qua(avp0(1:3)), avp0(4:6), avp0(7:9)); % avp分解
        [ins.qnb, ins.vn, ins.pos] = setvals(qnb0, vn0, pos0); % ins的avp记录（分开）
        [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);  % 四元数分解
        ins.Cnb0 = ins.Cnb; ins.vn0 = vn0; ins.pos0 = pos0; % 初始avp记录
        ins.avp  = [ins.att; ins.vn; ins.pos]; % ins的avp记录（总）
        % 地球参数
        ins.eth = ethinit(ins.pos, ins.vn);
    	% 'wib,web,fn,an,Mpv,MpvCnb,Mpvvn,CW' 计算一些外部参数 
        ins.wib = ins.Cnb'*ins.eth.wnin;
        ins.fn = -ins.eth.gn;  ins.fb = ins.Cnb'*ins.fn;
    	[ins.wnb, ins.web, ins.an] = setvals(zeros(3,1));
    	ins.Mpv = [0, 1/ins.eth.RMh, 0; 1/ins.eth.clRNh, 0, 0; 0, 0, 1];
        ins.MpvCnb = ins.Mpv*ins.Cnb;  ins.Mpvvn = ins.Mpv*ins.vn;
        % calibration parameters
    	[ins.Kg, ins.Ka] = setvals(eye(3)); 
        [ins.eb, ins.db] = setvals(zeros(3,1));
        [ins.tauG, ins.tauA] = setvals(inf(3,1)); % gyro & acc correlation time
        ins.lever = zeros(3,1); ins = inslever(ins); % lever arm
    	ins.tDelay = 0; % time delay
        ins.openloop = 0;
        glv.wm_1 = zeros(3,1)';  glv.vm_1 = zeros(3,1)';  % for 'single sample+previous sample' coning algorithm
        ins.an0 = zeros(3,1);  ins.anbar = ins.an0;
    case 'update'
        [ins,imu]=setvals(varargin);
        nn = size(imu,1); % 双子样
        nts = nn*ins.ts;  nts2 = nts/2;  ins.nts = nts;
        % nts = nn*ins.ts; nts2=ins.ts;
        [phim, dvbm] = cnscl(imu,0);    % 圆锥 划桨补偿
        phim = ins.Kg*phim-ins.eb*nts; dvbm = ins.Ka*dvbm-ins.db*nts;  % 对准
        % 地球参数角速度更新
        vn01 = ins.vn+ins.an*nts2; pos01 = ins.pos+ins.Mpv*vn01*nts2;  % extrapolation at t1/2
        if ins.openloop==0, ins.eth = ethupdate(ins.eth, pos01, vn01);
        elseif ins.openloop==1, ins.eth = ethupdate(ins.eth, ins.pos0, ins.vn0); end
        ins.wib = phim/nts; 
        ins.fb = dvbm/nts; 
        ins.web = ins.wib - ins.Cnb'*ins.eth.wnie;
        %     ins.wnb = ins.wib - ins.Cnb'*ins.eth.wnin;
        ins.wnb = ins.wib - (ins.Cnb*rv2m(phim/2))'*ins.eth.wnin;  % 2014-11-30
        % (1)速度更新
        ins.fn = qmulv(ins.qnb, ins.fb);
        % ins.an = qmulv(rv2q(-ins.eth.wnin*nts2),ins.fn) + ins.eth.gcc;
        ins.an = rotv(-ins.eth.wnin*nts2, ins.fn) + ins.eth.gcc;  
        ins.anbar = 0.9*ins.anbar + 0.1*ins.an;
        vn1 = ins.vn + ins.an*nts;
        % (2)位置更新
        %     ins.Mpv = [0, 1/ins.eth.RMh, 0; 1/ins.eth.clRNh, 0, 0; 0, 0, 1];
        ins.Mpv(4)=1/ins.eth.RMh; ins.Mpv(2)=1/ins.eth.clRNh;
        %     ins.Mpvvn = ins.Mpv*((ins.vn+vn1)/2+(ins.an-ins.an0)*nts^2/3);  % 2014-11-30
        ins.Mpvvn = ins.Mpv*(ins.vn+vn1)/2;
        ins.pos = ins.pos + ins.Mpvvn*nts;
        ins.vn = vn1;
        ins.an0 = ins.an;
        % (3)姿态更新
        ins.Cnb0 = ins.Cnb;
        % ins.qnb = qupdt(ins.qnb, ins.wnb*nts);  % lower accuracy than the next line
        ins.qnb = qupdt2(ins.qnb, phim, ins.eth.wnin*nts);
        [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
        ins.avp = [ins.att; ins.vn; ins.pos];
end

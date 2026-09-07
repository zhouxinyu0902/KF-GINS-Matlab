function dr = mydr(type,varargin)
%MYDR 包含初始化、推算功能
glvs
switch(type)
    case 'init'
        dr=[];
        if length(varargin)==3
            [pos0,dpos,ts]=setvals(varargin);
            beacon=[];
        elseif length(varargin)==4
            [pos0,dpos,ts,beacon]=setvals(varargin);
        end
        % dpos in m
        dr.kod=1;
        dr.pos=dxyz2pos(dpos',pos0)';
        dr.ts=ts;
        dr.beacon=beacon;
        dr.avp=[zeros(6,1);dr.pos];
    case 'update'
        if length(varargin)==4
            [dr,depther,compass,VXYZ_n]=setvals(varargin);
            dr.pos(3)=depther;
        elseif length(varargin)==3
            [dr,compass,VXYZ_n]=setvals(varargin);
        end
        VXYZ_n=VXYZ_n/dr.kod;
        if size(compass,2)==1
            Cn_b=a2mat([0,0,compass]);
            dr.vn=Cn_b*[VXYZ_n';0];
            dr.att=[0;0;compass];
        else
            Cn_b=a2mat(compass);
            dr.vn=Cn_b*[VXYZ_n';0];
            dr.att=compass';
        end
        % phi=compass;
        % VE=VXYZ_n(1)*cos(phi)-VXYZ_n(2)*sin(phi);
        % VN=VXYZ_n(1)*sin(phi)+VXYZ_n(2)*cos(phi);
        % dr.vn=[VE;VN;0];
        dSn=dr.vn(1:2)*dr.ts;
        dr.eth = earth(dr.pos, dr.vn);
        Mpv = [0, 1/dr.eth.RMh; 1/dr.eth.clRNh, 0];
        dr.ds=Mpv*dSn;
        dr.pos(1:2) = dr.pos(1:2) + dr.ds;
        dr.avp=[dr.att;dr.vn;dr.pos];
end
end


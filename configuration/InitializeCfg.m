% -------------------------------------------------------------------------
%  Author : zxy
%  Date : 2025.5.6
% -------------------------------------------------------------------------

function navstate = InitializeCfg(cfg)
% navigation state initialization
navstate.time = cfg.starttime;
navstate.pos = cfg.initpos;
navstate.vel = cfg.initvel;
navstate.att = cfg.initatt;
navstate.cbn = euler2dcm(cfg.initatt);
navstate.qbn = euler2quat(cfg.initatt);
navstate.gyrbias = cfg.initgyrbias;
navstate.accbias = cfg.initaccbias;
navstate.gyrscale = cfg.initgyrscale;
navstate.accscale = cfg.initaccscale;
param = Param();
[navstate.Rm, navstate.Rn] = getRmRn(cfg.initpos(1), param);
navstate.gravity = getGravity(cfg.initpos);
end
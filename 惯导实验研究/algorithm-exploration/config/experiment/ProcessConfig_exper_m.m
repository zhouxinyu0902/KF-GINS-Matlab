function cfg = ProcessConfig_exper_m(input_dir)
%PROCESSCONFIG_EXPER_M 米制位置误差状态的实测数据配置。
%   路径、数据集初值和文件解析与 ProcessConfig_exper 完全一致，仅把
%   初始位置标准差恢复为 [dN,dE,dD] 米制表示。

    if nargin < 1
        input_dir = [];
    end
    cfg = ProcessConfig_exper(input_dir);
    param = Param();
    [rm, rn] = getRmRn(cfg.initpos(1), param);
    DR = diag([rm + cfg.initpos(3), ...
        (rn + cfg.initpos(3))*cos(cfg.initpos(1)), -1]);
    cfg.initposstd = DR*cfg.initposstd;
end

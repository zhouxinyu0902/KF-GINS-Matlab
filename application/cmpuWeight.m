% clear;

% 几何参数 (单位：厘米)
% thickness = 2.2; % 厚度
% R1 = 20;       % 外半径
thickness = 3.5; % 厚度
R1 = 33.8/2;       % 外半径
R2 = R1 - thickness; % 内半径
h = 50;        % 高度

% 材料选择
% material = "7075-T6"; % 当前选择 7075-T6
material = "6061-T6";
% 根据材料选择弹性模量 E 和密度
if material == "6061-T6"
    E = 68900; % MPa
    density = 2.7; % g/cm^3 (贵)
elseif material == "7075-T6"
    density = 2.81; % g/cm^3
    E = 71700; % MPa
end

% 体积和质量计算
% 注意：R1, R2, h 都是 cm，density 是 g/cm^3，所以体积是 cm^3，质量是 g。
% 转换为 kg 需要除以 1000。
volomn = pi * (R1^2 - R2^2) * h; % cm^3
fprintf("体积（cm^3）,%f\n", volomn);
weight = volomn * density / 1000; % kg
fprintf("质量（kg）,%f\n", weight);

%% 临界长度
% 公式： L = 1.17 * R1 * 2 * sqrt(R1 * 2 / thickness)
% 这是一个经验性的临界长度公式，用于判断圆筒属于长、中长还是短。
L_critical = 1.17 * 2 * R1 * sqrt(2 * R1 / thickness); % 修正为平均直径 2*R1 或外径 2*R1
fprintf("临界长度（cm）,%f\n", L_critical); % 这里的 L 是指圆筒的高度 h
if h < L_critical
    fprintf("该圆筒属于短圆筒\n");
else
    fprintf("该圆筒属于长圆筒\n");
end


%% 临界厚度的计算
% 注意：这里 pa 是您设定的工作压力，不是临界压力。
% 两个公式中的 D0 是“圆柱平均半径”，我们近似取 R1。
% 转换为毫米 (mm) 进行计算，以便与 E 的 MPa 单位匹配更直接。

R1_mm = R1 * 10; % cm 转换为 mm
h_mm = h * 10;   % cm 转换为 mm
thickness_mm = thickness * 10; % cm 转换为 mm

Pa_MPa = 20; % 工作压力已经是 MPa

% 长圆筒临界厚度公式 (s1)
% s1 = D0 * (m * Ps / (2.2 * E))^(1/3)
% m 为安全系数，取 2。
m = 2;
thick_long_mm = R1_mm * (m * Pa_MPa / (2.2 * E))^(1/3); % R1_mm 作为 D0
fprintf("长圆筒临界厚度（cm）,%f\n", thick_long_mm / 10);

% 短圆筒临界厚度公式 (s2)
% s2 = D0 * (m * Ps * L / (2.2 * E * D0))^0.4
thick_short_mm = R1_mm * (m * Pa_MPa * h_mm / (2.2 * E * R1_mm))^0.4; % R1_mm 作为 D0
fprintf("短圆筒临界厚度（cm）,%f\n", thick_short_mm / 10);


%% 临界压力计算
% 这里计算的是基于厚度为 1cm (10mm) 的圆筒的临界压力。
% 公式：Pk = 2.6 * E * (S/D)^2.5 / (L/D)
% 注意：这里的 D 是直径，不是半径。我们统一用外径。
% S 是壁厚。

D_outer_mm = R1 * 2 * 10; % 外直径 (mm) = 2 * R1 (cm) * 10 mm/cm
S_wall_mm = thickness * 10; % 壁厚 (mm)
h_mm = h * 10; % 圆筒长度 (mm)

pp = 2.6 * E * (S_wall_mm / D_outer_mm)^2.5 / (h_mm / D_outer_mm);
fprintf("短圆筒临界压力（MPa）,%f\n", pp);
fprintf("安全系数,%f\n", pp/20);
%% 弹性屈曲公式 P_cr = (2 * E / (1 - mu^2)) * (t/D)^3
% 这里需要泊松比 mu。
if material == "6061-T6"
    mu = 0.33;
elseif material == "7075-T6"
    mu = 0.33; % 7075-T6 的泊松比也近似为 0.33
end

Pcr_elastic = (2 * E / (1 - mu^2)) * (S_wall_mm / D_outer_mm)^3;
fprintf("长圆筒弹性屈曲临界压力（MPa）,%f\n", Pcr_elastic);

22*268/2/(310/3.5)
20*378/2/(310/3.5)
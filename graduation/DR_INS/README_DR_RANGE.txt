DR_RANGE_code 第一版说明
========================

用途：
  DVL + 罗盘 + 深度计航位推算，单信标斜距EKF校正。

输入文件：
  dvl.txt       : t vb_x vb_y vb_z
  compass.txt   : t pitch roll yaw          单位 rad
  depth.txt     : t depth                   单位 m，NED中D向下为正
  beacon.txt    : t slant horizontal lat lon h，单位 s m m rad rad m
  reference.txt : t N E D pitch roll yaw vN vE vD，可选

运行步骤：
  1. 把本文件夹内所有 .m 文件放到 Matlab 当前路径或工程路径下。
  2. 修改 main_DR_RANGE.m 中的 in_dir。
  3. 修改 config_DR_RANGE.m 中的 cfg.lat0、cfg.lon0、cfg.depth0，使其与数据生成脚本一致。
  4. 运行 main_DR_RANGE。

主要输出：
  output_DR_RANGE/Origin-DR.nav
  output_DR_RANGE/DR-RANGE.nav
  output_DR_RANGE/DR-RANGE-state.txt
  output_DR_RANGE/DR-RANGE-innovation.txt
  output_DR_RANGE/Origin-DR-ned.txt
  output_DR_RANGE/DR-RANGE-ned.txt

状态定义：
  默认4维：x = [dK; dYaw; dLat; dLon]
  定义为 true - DR_estimate。
  因此 DRfeedback 中位置修正是 dr.pos = dr.pos + x_pos。

第一版限制：
  1. 不含RTS/线性平滑。
  2. 不做严格异步插值，只按最近DR历元触发距离更新。
  3. 默认只反馈位置，不反馈DVL刻度和航向，便于观察参数可观测性。

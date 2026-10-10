clear
% data_source = "simulation";          % "simulation" 或 "experiment"
% case_name = 'case-00';
data_source = "experiment";          % "simulation" 或 "experiment"
case_name = 'case-06';
for i=1:3
    phasestd = i*5;
    azistd = 0.4*phasestd/5;
    default_azimuth_std_deg = azistd;
    % run_range_azimuth_dropout_double_rts
    % run_double_hydrophone_forward_ekf
    % run_double_hydrophone_forward_ekf_adaptive
    run_double_hydrophone_forward_ekf_position_aided
end
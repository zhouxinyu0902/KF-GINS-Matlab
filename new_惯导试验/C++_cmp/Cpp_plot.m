truth='D:\Github\KF-GINS-main\dataset_exper6\input\truth.nav';
EKF = 'D:\Github\KF-GINS-main\dataset_exper6\output\Realtime_Nav.nav';
RTS='D:\Github\KF-GINS-main\dataset_exper6\output\RTS_Result_Single.nav';
RTS2='D:\Github\KF-GINS-main\dataset_exper6\output\RTS_Result_twice.nav';
[FIG,EXCEL]=calc_radial_error_gjb(truth,EKF,RTS2,RTS);
legend('EKF','RTS','RTS2');

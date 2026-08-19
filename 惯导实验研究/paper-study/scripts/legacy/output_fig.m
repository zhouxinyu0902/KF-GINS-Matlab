fig=openfig("D:\Github\KF-GINS-Matlab\new_惯导试验\simu\output\compare-none-single-double.fig");
legend("ESKF","Single-stage RTS","Proposed two-stage RTS");
xlabel('Time(s)')
ylabel("Radial Error(m)")
fig.Position = [100 100 350 350];
title("Dataset1 Navigation Error")
exportgraphics(fig,"D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs\simu_Dataset1.png", 'Resolution', 600);

fig=openfig("D:\Github\KF-GINS-Matlab\new_惯导试验\simu\output1\compare-none-single-double.fig");
legend("ESKF","Single-stage RTS","Proposed two-stage RTS");
xlabel('Time(s)')
ylabel("Radial Error(m)")
fig.Position = [100 100 350 350];
title("Dataset2 Navigation Error")
exportgraphics(fig,"D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs\simu_Dataset2.png", 'Resolution', 600);
%%

fig=openfig("D:\Github\KF-GINS-Matlab\new_惯导试验\output\output6_height_back\compare-none-assign-measUpdate.fig");
legend("No-height update","Direct assignment","Measurement update");
xlabel('Time(s)')
ylabel("Radial Error(m)")
fig.Position = [100 100 350 350];
exportgraphics(fig,"D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs\exper_compare-none-assign-measUpdate.png", 'Resolution', 600);


fig=openfig("D:\Github\KF-GINS-Matlab\new_惯导试验\output\output5\alt-B1-B2-B3\threewaysCMP.fig");
newlegend = {"ES-EKF-Alternating"',"ES-EKF-Fixed-B1","ES-EKF-Fixed-B2","ES-EKF-Fixed-B3"};
legend(newlegend)
title("Dataset1 ")
xlabel('Time(s)')
ylabel("Radial Error(m)")
fig.Position = [100 100 350 300];
exportgraphics(fig,"D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs\exper1_threewaysCMP.png", 'Resolution', 600);

fig=openfig("D:\Github\KF-GINS-Matlab\new_惯导试验\output\output6\alt-B1-B2-B3\threewaysCMP.fig");
newlegend = {"ES-EKF-Alternating"',"ES-EKF-Fixed-B1","ES-EKF-Fixed-B2","ES-EKF-Fixed-B3"};
legend(newlegend)
title("Dataset2 ")
xlabel('Time(s)')
ylabel("Radial Error(m)")
fig.Position = [100 100 350 300];
exportgraphics(fig,"D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs\exper2_threewaysCMP.png", 'Resolution', 600);

%%
fig=openfig("D:\Github\KF-GINS-Matlab\new_惯导试验\output\output5\补偿前后误差对比-rad-5.fig");
legend("ESKF","Single-stage RTS","Proposed two-stage RTS");
xlabel('Time(s)')
ylabel("Radial Error(m)")
fig.Position = [100 100 350 300];
title("Dataset1 Navigation Error")
exportgraphics(fig,"D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs\exper_Dataset1.png", 'Resolution', 600);

fig=openfig("D:\Github\KF-GINS-Matlab\new_惯导试验\output\output6\补偿前后误差对比-rad-6.fig");
legend("ESKF","Single-stage RTS","Proposed two-stage RTS");
xlabel('Time(s)')
ylabel("Radial Error(m)")
fig.Position = [100 100 350 300];
title("Dataset2 Navigation Error")
exportgraphics(fig,"D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE\figs\exper_Dataset2.png", 'Resolution', 600);
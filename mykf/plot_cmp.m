function plot_cmp(testnavpath,truthpath)
navdata = importdata(testnavpath);
truthdata = importdata(truthpath);
figure,
plot_pos(truthdata)
hold on
plot_pos(navdata)
end
function plot_pos(navdata)
% position
param = Param();
blh = navdata(:, 3:5);
blh(:, 1) = blh(:, 1) * param.D2R;
blh(:, 2) = blh(:, 2) * param.D2R;
first_blh = blh(1, 1:3);

[rm, rn] = getRmRn(first_blh(1), param);
h = first_blh(2);
DR = diag([rm + h, (rn + h)*cos(first_blh(1)), -1]);

% blh to ned
pos = zeros(size(blh));
for i = 1:size(pos, 1)
    delta_blh = blh(i, :) - first_blh;
    delta_pos = DR * delta_blh';
    pos(i, :) = delta_pos';
end

%% plane position
plot(pos(1, 2), pos(1, 1),'*');hold on
plot(pos(:, 2), pos(:, 1));
title('Position');
xlabel('East[m]');
ylabel('North[m]');
grid("on");

end
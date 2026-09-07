function updownlabel
%UPDOWNLABEL 此处显示有关此函数的摘要
%   此处显示详细说明
ylim_vals = ylim;
ii=[4088 4352 4752 4992 6192 6448 6784 7040 8472 8728]-72;
for i=1:2:9
    hold on
    plot([ii(i) ii(i)], ylim_vals,  'Color', [0 0 0 0.4],'LineStyle','--');
    hold on
    plot([ii(i+1) ii(i+1)], ylim_vals, 'Color', [0 0 0 0.4],'LineStyle','--');
end
end


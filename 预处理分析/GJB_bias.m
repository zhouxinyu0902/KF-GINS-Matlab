function bias = GJB_bias(omega,latitude,type)
% 假设数据存储在向量omega中，单位为°/h
switch(type)
    case 'X'
        bias = mean(omega); %东向
    case 'Y'
        bias = mean(omega) - (15.041 * cosd(latitude)); % 地球自转角速度15.041°/h %北向
    case 'Z'
        bias = mean(omega) - (15.041 * sind(latitude)); % 地球自转角速度15.041°/h %天向
end
end


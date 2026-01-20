function [c, ceq] = circle_constraint(pb)
        % 非线性不等式约束 c(x) <= 0
        % 信标到起点的距离平方 - R_region^2 <= 0
        c = pb(1)^2 + pb(2)^2 - 4000^2;
        % 非线性等式约束 ceq(x) = 0 (本例无等式约束)
        ceq = [];
    end
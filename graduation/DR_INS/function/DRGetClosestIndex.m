function idx = DRGetClosestIndex(data, t, idx0)
%DRGETCLOSESTINDEX 在时间递增数据中寻找离t最近的行索引

    if nargin < 3 || isempty(idx0)
        idx0 = 1;
    end

    n = size(data, 1);
    idx = max(1, min(idx0, n));

    while idx < n && data(idx+1, 1) <= t
        idx = idx + 1;
    end

    if idx < n
        if abs(data(idx+1,1) - t) < abs(data(idx,1) - t)
            idx = idx + 1;
        end
    end
end

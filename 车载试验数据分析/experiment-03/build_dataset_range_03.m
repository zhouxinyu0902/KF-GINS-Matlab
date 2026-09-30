function build_dataset_range_03(dataset_ids, show_figures)
%BUILD_DATASET_RANGE_03 构造120/830评估距离并保留原始实测距离。
%
% 与experiment-01不同，本试验不生成虚拟range1~range3：
%   range.txt     原始实测距离，导航使用；
%   range_120.txt 原测距时刻/信标位置 + 120位置，仅评估；
%   range_830.txt 原测距时刻/信标位置 + 830位置，仅评估。

    if nargin < 1 || isempty(dataset_ids)
        dataset_ids = 1:3;
    end
    if nargin < 2 || isempty(show_figures)
        show_figures = false;
    end
    process_data_1(dataset_ids, show_figures);
end

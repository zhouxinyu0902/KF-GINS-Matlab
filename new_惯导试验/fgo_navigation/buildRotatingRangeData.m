function [rangedata, beacon_ids] = buildRotatingRangeData( ...
        range_streams, start_time, end_time, opts)
%BUILDROTATINGRANGEDATA Select one range every 7 minutes and rotate beacons.

    switch lower(opts.range_selection_mode)
        case 'legacy_rows'
            step = round(opts.range_interval_sec);
            sampled = cell(size(range_streams));
            count = inf;
            for i = 1:length(range_streams)
                sampled{i} = range_streams{i}(step:step:end, :);
                count = min(count, size(sampled{i}, 1));
            end
            if count < 1
                rangedata = zeros(0, size(range_streams{1}, 2));
                beacon_ids = zeros(0, 1);
                return;
            end

            rangedata = zeros(count, size(sampled{1}, 2));
            beacon_ids = zeros(count, 1);
            for row = 1:count
                beacon_id = mod(row - 1, length(sampled)) + 1;
                rangedata(row, :) = sampled{beacon_id}(row, :);
                beacon_ids(row) = beacon_id;
            end

        case 'timestamp'
            stream_start = cellfun(@(x) x(1, 1), range_streams);
            stream_end = cellfun(@(x) x(end, 1), range_streams);
            first_time = max([start_time + opts.range_interval_sec, stream_start]);
            final_time = min([end_time, stream_end]);
            targets = (first_time:opts.range_interval_sec:final_time)';

            rangedata = zeros(length(targets), size(range_streams{1}, 2));
            beacon_ids = zeros(length(targets), 1);
            for row = 1:length(targets)
                beacon_id = mod(row - 1, length(range_streams)) + 1;
                stream = range_streams{beacon_id};
                [error_sec, idx] = min(abs(stream(:, 1) - targets(row)));
                stream_dt = median(diff(stream(:, 1)));
                if error_sec > max(1, 2 * stream_dt)
                    error('No range sample close to target time %.3f.', targets(row));
                end
                rangedata(row, :) = stream(idx, :);
                beacon_ids(row) = beacon_id;
            end

        otherwise
            error('Unknown range_selection_mode: %s', opts.range_selection_mode);
    end

    [~, order] = sort(rangedata(:, 1));
    rangedata = rangedata(order, :);
    beacon_ids = beacon_ids(order);
end

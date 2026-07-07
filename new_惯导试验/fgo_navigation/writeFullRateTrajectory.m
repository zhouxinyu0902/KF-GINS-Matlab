function writeFullRateTrajectory(filepath, imudata, nav0, ...
        node_times, node_errors, opts)
%WRITEFULLRATETRAJECTORY Apply interpolated graph errors to the 100 Hz INS.

    errors_at_imu = interp1(node_times, node_errors', imudata(:, 1), ...
        'linear', 'extrap')';

    param = Param();
    fp = fopen(filepath, 'wt');
    if fp < 0
        error('Could not open output file: %s', filepath);
    end
    cleanup = onCleanup(@() fclose(fp));

    nav = nav0;
    corrected = applyErrorCorrection(nav, errors_at_imu(:, 1), ...
        opts.apply_attitude_correction);
    writeOne(fp, corrected, 1, param);

    for k = 2:size(imudata, 1)
        nav = InsMech(nav, imudata(k - 1, :)', imudata(k, :)');
        corrected = applyErrorCorrection(nav, errors_at_imu(:, k), ...
            opts.apply_attitude_correction);
        writeOne(fp, corrected, k, param);
    end
end

function writeOne(fp, nav, sequence, param)
    row = [sequence; nav.time; ...
        nav.pos(1) * param.R2D; nav.pos(2) * param.R2D; nav.pos(3); ...
        nav.vel; nav.att * param.R2D];
    fprintf(fp, ['%6d %12.6f %14.9f %14.9f %10.4f ', ...
        '%10.5f %10.5f %10.5f %10.6f %10.6f %10.6f\n'], row);
end

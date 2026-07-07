function writeNavStates(filepath, navstates)
%WRITENAVSTATES Write navigation states in the existing .nav format.

    param = Param();
    fp = fopen(filepath, 'wt');
    if fp < 0
        error('Could not open output file: %s', filepath);
    end
    cleanup = onCleanup(@() fclose(fp));

    for k = 1:length(navstates)
        writeOne(fp, navstates{k}, k, param);
    end
end

function writeOne(fp, nav, sequence, param)
    row = [sequence; nav.time; ...
        nav.pos(1) * param.R2D; nav.pos(2) * param.R2D; nav.pos(3); ...
        nav.vel; nav.att * param.R2D];
    fprintf(fp, ['%6d %12.6f %14.9f %14.9f %10.4f ', ...
        '%10.5f %10.5f %10.5f %10.6f %10.6f %10.6f\n'], row);
end

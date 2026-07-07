function checkFgoDependencies()
%CHECKFGODEPENDENCIES Fail early when the host INS project is not on path.

    required = { ...
        'config_1', 'myInitialize_15state', 'myInsPropagate_15state', ...
        'InsMech', 'Param', 'getRmRn', 'getGravity', ...
        'rotvec2quat', 'quatProd', 'quatNormalized', ...
        'quat2dcm', 'dcm2euler'};

    missing = {};
    for i = 1:length(required)
        if exist(required{i}, 'file') ~= 2
            missing{end + 1} = required{i}; %#ok<AGROW>
        end
    end

    if ~isempty(missing)
        error('Missing required functions on MATLAB path: %s', ...
            strjoin(missing, ', '));
    end
end

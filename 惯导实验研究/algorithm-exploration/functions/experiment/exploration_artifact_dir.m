function artifact_dir = exploration_artifact_dir(navigation_dir)
%EXPLORATION_ARTIFACT_DIR 将导航结果目录映射到同层级图表目录。
    navigation_dir = char(string(navigation_dir));
    marker = [filesep, 'navigation-results', filesep];
    replacement = [filesep, 'figures-tables', filesep];
    if contains(navigation_dir, marker)
        artifact_dir = strrep(navigation_dir, marker, replacement);
    else
        error('路径不属于algorithm-exploration/navigation-results：%s', ...
            navigation_dir);
    end
    if ~isfolder(artifact_dir)
        mkdir(artifact_dir);
    end
end

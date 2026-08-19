function artifact_dir = paper_artifact_dir(navigation_dir)
%PAPER_ARTIFACT_DIR 将论文导航结果目录映射到同层级图表目录。
    navigation_dir = char(string(navigation_dir));
    marker = [filesep, 'navigation-results', filesep];
    replacement = [filesep, 'figures-tables', filesep];
    if contains(navigation_dir, marker)
        artifact_dir = strrep(navigation_dir, marker, replacement);
    else
        error('路径不属于paper-study/navigation-results：%s', navigation_dir);
    end
    if ~isfolder(artifact_dir)
        mkdir(artifact_dir);
    end
end

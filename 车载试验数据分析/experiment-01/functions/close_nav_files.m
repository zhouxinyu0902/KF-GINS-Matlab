function close_nav_files(file_ids)
%CLOSE_NAV_FILES 安全关闭仍处于打开状态的导航结果文件。
% 配合 onCleanup 使用，保证脚本异常退出时不遗留 Windows 文件锁。

    for file_id = file_ids(:)'
        if file_id > 0 && ~isempty(fopen(file_id))
            fclose(file_id);
        end
    end
end

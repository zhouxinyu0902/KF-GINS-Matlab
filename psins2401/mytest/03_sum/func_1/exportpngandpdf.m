function exportpngandpdf(fig, path)
exportgraphics(fig, [path,'.png'], 'Resolution', 600);
exportgraphics(fig, [path,'.pdf'], "ContentType", "vector");
end
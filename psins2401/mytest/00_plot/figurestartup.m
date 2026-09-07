function figurestartup(width,height,alw,fsz,lw,msz)
% % % Defaults for figures
% width = 3;     % Width in inches
% height = 3;    % Height in inches
% alw = 0.75;    % AxesLineWidth    Paper=0.75; Presentation=1 
% fsz = 8;      % Fontsize          Paper=8; Presentation=14  
% lw = 1.5;      % LineWidth        Paper=1.5; Presentation=2     
% msz = 8;       % MarkerSize       Paper=8; Presentation=12

%% Text Size
set(0,'DefaultAxesFontsize',fsz);
set(0,'DefaultTextFontsize',fsz);
% set(0,'DefaultAxesFontWeight','bold');
% set(0,'DefaultTextFontWeight','bold');

%% Text Fonts
set(0,'DefaultTextFontname','Arial')
set(0,'DefaultAxesFontname','Arial')
% set(0,'DefaultTextFontname','Times New Roman')
% set(0,'DefaultAxesFontname','Times New Roman')

% The properties we've been using in the figures
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultAxesLineWidth',alw); % set the default line marker size to msz

% Set the default Size for display
defpos = get(0,'defaultFigurePosition');
set(0,'defaultFigurePosition', [defpos(1) defpos(2) width*100, height*100]);

% Set the defaults for saving/printing to a file
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
defsize = get(gcf, 'PaperSize');
left = (defsize(1)- width)/2;
bottom = (defsize(2)- height)/2;
defsize = [left, bottom, width, height];
set(0, 'defaultFigurePaperPosition', defsize);

end
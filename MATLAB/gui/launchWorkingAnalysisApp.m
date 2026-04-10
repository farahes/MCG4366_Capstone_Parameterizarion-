function app = launchWorkingAnalysisApp()
% Launch WorkingAnalysisApp with tabs visible at startup.
%
% This avoids direct package edits to the .mlapp file.

    rootDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootDir);
    addpath(fullfile(rootDir, 'analysis'));
    addpath(fullfile(rootDir, 'components'));
    addpath(fullfile(rootDir, 'gui'));

    app = WorkingAnalysisApp;

    tg = findobj(app.UIFigure, 'Tag', 'AppTabGroup');
    if isempty(tg)
        figW = app.UIFigure.Position(3);
        figH = app.UIFigure.Position(4);

        tg = uitabgroup(app.UIFigure, ...
            'Position', [0 0 figW figH], ...
            'Tag', 'AppTabGroup');

        tabR = uitab(tg, 'Title', 'Calculated Dimensions', 'Tag', 'TabResults');
        tabP = uitab(tg, 'Title', 'Gait Cycle Plots', 'Tag', 'TabPlots');
        tabL = uitab(tg, 'Title', 'Log File', 'Tag', 'TabLog');

        kids = app.UIFigure.Children;
        for k = 1:numel(kids)
            if ~isequal(kids(k), tg)
                kids(k).Parent = tabR;
            end
        end

        % Add logo to the first tab only (top-right corner).
        logoPath = fullfile(rootDir, 'gui', 'knee_logo.png');
        if isfile(logoPath)
            logoW = min(190, max(110, round(0.15 * figW)));
            logoH = 84;
            try
                info = imfinfo(logoPath);
                logoH = max(28, round(logoW * (info.Height / info.Width)));
            catch
                % Keep fallback height when image metadata is unavailable.
            end

            tabHeaderH = 34;
            tabContentH = max(120, figH - tabHeaderH);
            margin = 6;
            logoX = 828;
            logoY = max(margin, tabContentH - logoH - margin);

            logoImg = uiimage(tabR, ...
                'Position', [logoX logoY logoW logoH], ...
                'ImageSource', logoPath, ...
                'Tag', 'TopRightLogo');

            if isprop(logoImg, 'ScaleMethod')
                logoImg.ScaleMethod = 'fit';
            end
        end

        labelW = max(100, figW - 60);
        labelFontSize = 24;
        labelH = 44;
        labelX = max(10, (figW - labelW)/2);
        labelY = max(10, (figH - labelH)/2);

        uilabel(tabP, ...
            'Position', [labelX labelY labelW labelH], ...
            'HorizontalAlignment', 'center', ...
            'FontSize', labelFontSize, ...
            'FontWeight', 'bold', ...
            'Text', 'Press Generate to run analysis and populate plots.');

        uilabel(tabL, ...
            'Position', [labelX labelY labelW labelH], ...
            'HorizontalAlignment', 'center', ...
            'FontSize', labelFontSize, ...
            'FontWeight', 'bold', ...
            'Text', 'Press Generate to run analysis and load the log file.');

        tg.SelectedTab = tabR;
    end
end

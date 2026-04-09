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

        uilabel(tabP, ...
            'Position', [30 30 520 22], ...
            'Text', 'Press Generate to run analysis and populate plots.');

        uilabel(tabL, ...
            'Position', [30 30 520 22], ...
            'Text', 'Press Generate to run analysis and load the log file.');
    end
end

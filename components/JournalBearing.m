classdef JournalBearing

methods (Static)

% Load and cache the journal bearing catalogue from JournalBearingSizes.csv.
% 'persistent' ensures the CSV is read only once per MATLAB session,
% avoiding repeated disk reads when this function is called multiple times.
function journalBearingTable = getJBTable()
    persistent jbTable
    if isempty(jbTable)
        rootDir = fileparts(fileparts(which('JournalBearing')));
        jbTable = readtable(fullfile(rootDir, 'data', 'JournalBearingSizes.csv'));
    end
    journalBearingTable = jbTable;
end

% Select the smallest standard journal bearing satisfying both constraints:
%   (1) Bore ID  ≥  required shaft diameter  (D_shaft)
%   (2) Dynamic radial load capacity  >  applied JRF
%
% Searches the McMaster-Carr catalogue linearly from smallest to largest.
% Load ratings in the CSV are in lbf and are converted to N (1 lbf = 4.448 N).
% Returns the first qualifying entry; raises an error if none is found.
function journalDimensions = getJournalBearingDimensions(JRF,D_shaft)

    jbTable = JournalBearing.getJBTable();
    shaftDCol = matlab.lang.makeValidName('For Shaft Dia., mm');
    shaftD = jbTable.(shaftDCol);
    houseIDCol = matlab.lang.makeValidName('For Housing ID, mm');
    houseID = jbTable.(houseIDCol);
    lngthCol = matlab.lang.makeValidName('Lg., mm');
    lngth = jbTable.(lngthCol);
    flangetCol = matlab.lang.makeValidName('Flange Thk., mm');
    flanget = jbTable.(flangetCol);
    radialLoadCol = matlab.lang.makeValidName('Dynamic Radial Load at 120rpm (lb)');
    radialLoad = jbTable.(radialLoadCol);
    partCol = matlab.lang.makeValidName('Part No.');
    part = jbTable.(partCol);

    for i = 1:length(shaftD)
        if shaftD(i)/1000 >= D_shaft
            maxRadialLoad = radialLoad(i)*4.44822162;   % convert lbf to N
            if maxRadialLoad > JRF
                journalDimensions.housingID = houseID(i)/1000;
                journalDimensions.shaftD = shaftD(i)/1000;
                journalDimensions.lngth = lngth(i)/1000;
                journalDimensions.ft = flanget(i)/1000;
                journalDimensions.prt = part(i);
                return
            end
        end
    end

    % if not standard journal bearing suits the application
    error('JournalBearing:NoSuitableBearing', 'No standard journal bearing is suitable for JRF = %.2f N and shaft diameter = %.2f mm.', JRF, D_shaft);

end

end

end
classdef JournalBearing

methods (Static)

function journalBearingTable = getJBTable()
    persistent jbTable
    if isempty(jbTable)
        jbTable = readtable('JournalBearingSizes.csv');
    end
    journalBearingTable = jbTable;
end

function journalDimensions = getJournalBearingDimensions(JRF,D_shaft)

    jbTable = JournalBearing.getJBTable();
    shaftDCol = matlab.lang.makeValidName('For Shaft Dia., mm');
    shaftD = jbTable.(shaftDCol);
    houseIDCol = matlab.lang.makeValidName('For Housing ID, mm');
    houseID = jbTable.(houseIDCol);
    lngthCol = matlab.lang.makeValidName('Lg., mm');
    lngth = jbTable.(lngthCol);
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
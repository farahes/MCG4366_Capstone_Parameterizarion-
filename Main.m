%{

Subscript Convention
f : foot
l : leg
a : ankle
k : knee
s : shaft
su : supports
fr: frame
lp : lip
b : ball
py : pyramid adpater

%}

classdef Main

properties (Constant)

    % SAFETY FACTORS
    n_frame = 2;
    n_shaft = 2.5;

    % SHAFT MATERIALS

    % 1045 HR Carbon Steel
    Sy_s = 310*1000000; % [Pa] yield strength
    Su_s = 565*1000000; % [Pa] ultimate strength
    E_s = 200*1000000000; % [Pa] Young's Modulus
    nu_s = 0.29;   % Poisson's ratio

    % FRAME MATERIALS

    % 6061 T6 Aluminum
    Sy_fr = 276*1000000;    % [Pa] yield strength
    Su_fr = 310*1000000;    % [Pa] ultimate strength
    E_fr = 68.9*1000000000; % [Pa] Young's Modulus
    nu_fr = 0.33;   % Poisson's ratio
    %{
    % 1010 Carbon Steel
    Sy_fr = 365*1000000;    % [Pa] yield strength
    Su_fr = 305*1000000;    % [Pa] ultimate strength
    E_fr = 190*1000000000; % [Pa] Young's Modulus
    nu_fr = 0.27;   % Poisson's ratio
    %}

    % PYRAMID ADAPTER MATERIAL

    % Ti-6Al-4V
    Sy_py = 880; % [MPa] yield strength 

end
    
methods (Static)

% Shear yield stress
function Ssy = getSsy(Sy)
    Ssy = 0.58*Sy;   % [MPa]
end

% Shear modulus
function G = getG(E, nu)
    G = E/(2*(1+nu));
end

% --------DISPLAY FUNCTIONS--------

% Create array of results to display to GUI
function displayTable = displayResults(results)

    displayTable = table( ...
        {
            'D_s'
            'd_s'
            'L_s'
            'l_s'
            'Journal bearing'
            'w_k'
            'd_isu'
            't_su'
            'L_fr'
            'a'
            'b'
            't_lp'
            'd_valve'
            'Q'
            'Cv'
            'n_upper_py'
            'n_lower_py'
        }, ...
        {
            'Large diamer of the shaft'
            'Small diameter of the shaft'
            'Length of large diameter section of the shaft'
            'Total length of the shaft'
            'Journal bearing specifications'
            'Width of the square key'
            'Inner diameter of the supports'
            'Thickness of the supports'
            'Length of the frame legs'
            'Frame leg cross section, long dimension (width)'
            'Frame leg cross section, short dimension (thickness)'
            'Thickness of the lip'
            'Optimal valve diameter'
            'Flow rate'
            'Cv value for valve'
            'Safety factor for upper pyramid adapter'
            'Saftey factor for lower pyramid adapter'
        }, ...
        {
            results.D_s
            results.d_s
            results.L_s 
            results.l_s
            sprintf('ID: %.2f, OD: %.2f, Length: %.2f, Part no. %s', results.JBid, results. JBod, results.JBl, results.JBpart)
            results.w_k
            results.d_isu 
            results.t_su 
            results.L_fr 
            results.a 
            results.b 
            results.t_lp
            results.d_valve
            results.Q
            results.Cv
            results.n_upper_py
            results.n_lower_py
        }, ...
        {
            'mm'
            'mm'
            'mm'
            'mm'
            'mm (diameters and length)'
            'mm'
            'mm'
            'mm'
            'mm'
            'mm'
            'mm'
            'mm'
            'mm'
            'L/min'
            'N/A'
            'N/A'
            'N/A'
        }, ...
        'VariableNames', {'Variable Name','Description','Value','Units'} ...
    );

end

% /***************************************/
% /                 MAIN                  /
% /***************************************/

function results = getResults(BW, H)
    
    % create log file
    log = fopen('AnalysisLogFile.txt', 'w');
    if log == -1
        error('Could not create log file');
    end
    fprintf(log, '/***************************************/\n');
    fprintf(log, '/          WORKING ANALYSIS LOG         /\n');
    fprintf(log, '/***************************************/\n\n');

    % print patient data
    fprintf(log, 'Patient body weight: %.2f kg\n', BW);
    fprintf(log, 'Patient height: %.2f m\n\n', H);    

    % get the maximum JRF
    fprintf(log, 'Maximum Joint Reaction Force and Moment:\n');
    JRF = JointReactionForce.getMaxJRF(BW, H);
    F_kx = JRF.Fx;
    F_ky = JRF.Fy;
    F_k = sqrt(F_kx^2 + F_ky^2);
    M_k = JRF.M;
    fprintf(log, 'JRF = %.2f N\n', F_k);
    fprintf(log, 'JRF_x = %.2f N\n', F_kx);
    fprintf(log, 'JRF_y = %.2f N\n', F_ky);
    fprintf(log, 'JRM = %.2f Nm\n\n', M_k);

    fprintf(log, '-------- SHAFT ANALYSIS --------:\n\n');

    % get shaft dimensions
    rawDiameter = ShaftAnalysis.getShaftDiameter( ...
        log, ...
        Main.n_shaft, ...
        Main.Su_s, ...
        Main.Sy_s, ...
        Main.getSsy(Main.Sy_s), ...
        F_kx, ...
        F_ky, ...
        M_k ...
        );

    % get journal bearing specs
    fprintf(log, 'Journal bearing for inner diameter of %.2fmm:\n', rawDiameter.d*1000);
    JB = ShaftAnalysis.getJB(F_k, rawDiameter.d);
    fprintf(log, 'ID: %.2fmm, OD: %.2fmm, Length: %.2fmm, Part no. %s\n\n', JB.id*1000, JB.od*1000, JB.l*1000, JB.part{1});

    % get adjusted shaft dimensions
    adjDiameter = ShaftAnalysis.getFinalShaftDimensions(JB);
    fprintf(log, 'Adjusted shaft dimensions after journal bearing selection:\n');
    D_s = rawDiameter.D;
    d_s = adjDiameter.d_s;
    fprintf(log, 'D_s: %.2f mm\n', D_s*1000);
    fprintf(log, 'd_s: %.2f mm\n', d_s*1000);
    fprintf(log, 'L_s: %.2f mm\n', ShaftAnalysis.L_s*1000);
    fprintf(log, 'l_s: %.2f mm\n\n', adjDiameter.l_s*1000);

    % check torsional deflection
    while ~ShaftAnalysis.getCheckTorsion(log, M_k, Main.getG(Main.E_s, Main.nu_s), ShaftAnalysis.L_s, D_s, adjDiameter.l_s-ShaftAnalysis.L_s, d_s)
        D_s = D_s + 0.001;
        d_s = d_s + 0.001;
        JB = ShaftAnalysis.getJB(F_k, d_s);
        adjDiameter = ShaftAnalysis.getFinalShaftDimensions(JB);
    end

    D_s = ceil(rawDiameter.D*1000)/1000;    % round diameter up to the nearest mm
    fprintf(log, '\nFinal adjusted shaft dimensions:\n');
    fprintf(log, 'D_s: %.2f mm\n', D_s*1000);
    fprintf(log, 'd_s: %.2f mm\n', d_s*1000);
    fprintf(log, 'L_s: %.2f mm\n', ShaftAnalysis.L_s*1000);
    fprintf(log, 'l_s: %.2f mm\n\n', adjDiameter.l_s*1000);

    % get key dimension
    w_k = ShaftAnalysis.getWk(D_s);
    fprintf(log, 'Width of key: %.2f mm\n\n', w_k*1000);

    fprintf(log, '-------- FRAME ANALYSIS --------:\n\n');

    % get support thickness
    t_s = FrameAnalysis.getSupportThickness(log, Main.n_frame, F_k, Main.E_fr, Main.getSsy(Main.Sy_fr), Main.Sy_fr, JB.od);

    % get leg cross-section dimensions
    legDimensions = FrameAnalysis.getLegDimensions(log, Main.n_frame, F_k, Main.E_fr, Main.Sy_fr, Main.getSsy(Main.Sy_fr), JB.od);

    % get lip thickness
    F_lp = FrameAnalysis.getFlp(M_k, F_k);
    t_lp = FrameAnalysis.getLipThicknessComp(F_lp, FrameAnalysis.getSigma(Main.Sy_fr, Main.n_frame), FrameAnalysis.w_ball);
    fprintf(log, '\nLip Thickness:\n');
    fprintf(log, 'F_lp: %.2f N\n', F_lp);
    fprintf(log, 't_lp: %.2f mm\n', t_lp*1000);

    % get needle valve calculations and plot
    valve = HydraulicNeedleValve.getValveSize();
    HydraulicNeedleValve.plot(valve);

    % get pyramid adapter safety factors
    FoS_yield_u = PyramidAdapter.getFoS_yield_u(F_kx, F_ky, Main.Sy_py);
    FoS_yield_b = PyramidAdapter.getFoS_yield_b(F_kx, F_ky, Main.Sy_py);

    results.D_s = D_s*1000;
    results.d_s = adjDiameter.d_s*1000;
    results.L_s = ShaftAnalysis.L_s*1000;
    results.l_s = adjDiameter.l_s*1000;
    results.JBid = JB.id*1000;
    results. JBod = JB.od*1000;
    results.JBl = JB.l*1000;
    results.JBpart = JB.part{1};
    results.w_k = w_k*1000;
    results.d_isu = JB.od*1000;
    results.t_su = t_s*1000;
    results.L_fr = FrameAnalysis.L_fr*1000;
    results.a = legDimensions.a*1000;
    results.b = legDimensions.b*1000;
    results.t_lp = t_lp*1000;
    results.d_valve = valve.d_best_mm;
    results.Q = valve.Q_Lmin;
    results.Cv = valve.Cv;
    results.n_upper_py = FoS_yield_u;
    results.n_lower_py = FoS_yield_b;

    displayTable = Main.displayResults(results);    % display results to GUI
    results = displayTable;

    fclose(log);

end

end
end
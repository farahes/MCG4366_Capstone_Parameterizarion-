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

% Main is the top-level entry point for the prosthetic knee parameterization tool.
% Call Main.getResults(BW, H) with patient body weight [kg] and height [m] to run
% the full analysis pipeline and receive a results table for the GUI.
%
% Analysis sequence:
%   1. Knee joint reaction force & moment  — JointReactionForce (inverse dynamics)
%   2. Shaft diameter sizing               — ShaftAnalysis (fatigue, bending, shear)
%   3. Journal bearing selection           — JournalBearing (McMaster-Carr catalogue lookup)
%   4. Frame support & leg cross-section   — FrameAnalysis (buckling, shear, bending)
%   5. Needle valve sizing & flow plot     — HydraulicNeedleValve
%   6. Hydraulic pin diameter              — HydrPin (static bending & shear)
%   7. Pyramid adapter safety factors      — PyramidAdapter (von Mises)
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

% Shear yield stress via Von Mises (distortion-energy) criterion: Ssy ≈ 0.577*Sy.
% The value 0.58 is a common conservative approximation used in shaft design.
function Ssy = getSsy(Sy)
    Ssy = 0.58*Sy;   % [MPa]
end

% Shear modulus derived from Young's modulus and Poisson's ratio.
% From isotropic elasticity:  G = E / (2*(1+nu))
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
            'd_pin'
            'l_pin_upper'
            'l_pin_lower'
            'D_cyl'
            'delta_p_req'
            'Q_max_hyd'
            'L_restriction'
            'F_needle'
            'stroke_cyl'
            'L_cyl_min'
            'L_cyl_max'
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
            'Hydraulic pin diameter'
            'Upper hydraulic pin length'
            'Lower hydraulic pin length'
            'Hydraulic cylinder bore diameter (standard)'
            'Required cylinder pressure'
            'Maximum hydraulic flow rate'
            'Hagen-Poiseuille restriction channel length'
            'Minimum servo needle axial force'
            'Cylinder piston stroke'
            'Cylinder retracted length at min gait flexion'
            'Cylinder extended length at max gait flexion'
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
            results.d_pin
            results.l_pin_upper
            results.l_pin_lower
            results.D_cyl
            results.delta_p_req
            results.Q_max_hyd
            results.L_restriction
            results.F_needle
            results.stroke_cyl
            results.L_cyl_min
            results.L_cyl_max
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
            'mm'
            'mm'
            'mm'
            'mm'
            'MPa'
            'mL/s'
            'mm'
            'N'
            'mm'
            'mm'
            'mm'
        }, ...
        'VariableNames', {'Variable Name','Description','Value','Units'} ...
    );

end

% /***************************************/
% /                 MAIN                  /
% /***************************************/

function results = getResults(BW, H)

    % Add subfolders to MATLAB path so class files can be found
    rootDir = fileparts(which('Main'));
    addpath(fullfile(rootDir, 'analysis'));
    addpath(fullfile(rootDir, 'components'));

    % create log file
    log = fopen(fullfile(rootDir, 'logs', 'AnalysisLogFile.txt'), 'w');
    if log == -1
        error('Could not create log file');
    end
    fprintf(log, '/***************************************/\n');
    fprintf(log, '/          WORKING ANALYSIS LOG         /\n');
    fprintf(log, '/***************************************/\n\n');

    % print patient data
    fprintf(log, 'Patient body weight: %.2f kg\n', BW);
    fprintf(log, 'Patient height: %.2f m\n\n', H);    

    % get the maximum JRF and the full gait time-series (used for all 6 plots)
    fprintf(log, 'Maximum Joint Reaction Force and Moment:\n');
    JRF = JointReactionForce.getMaxJRF(BW, H);
    gaitSeries = JointReactionForce.getGaitTimeSeries(BW, H);  % all-frame data for plotting
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

    % get full hydraulic unit sizing (cylinder bore, flow rate, fluid properties, restriction length)
    fprintf(log, '-------- HYDRAULIC UNIT ANALYSIS --------:\n\n');
    hydraulics = HydraulicAnalysis.getResults(valve.d_best_mm / 1000);
    fprintf(log, 'Max load force:          %.2f N\n',   hydraulics.F_load_max_N);
    fprintf(log, 'Max piston velocity:     %.4f m/s\n', hydraulics.v_p_max_ms);
    fprintf(log, 'Cylinder D_min (calc):   %.2f mm\n',  hydraulics.D_min_calc_mm);
    fprintf(log, 'Cylinder D (standard):   %.0f mm\n',  hydraulics.D_standard_mm);
    fprintf(log, 'Required pressure:       %.2f MPa\n', hydraulics.delta_p_req_MPa);
    fprintf(log, 'Max flow rate:           %.2f mL/s\n',hydraulics.Q_max_mLs);
    fprintf(log, 'Oil density at 30C:      %.2f kg/m3\n', hydraulics.rho_30_kgm3);
    fprintf(log, 'Dynamic viscosity:       %.4f Pa.s\n',  hydraulics.mu_Pas);
    fprintf(log, 'Restriction length L:    %.2f mm\n',  hydraulics.L_restriction_mm);
    fprintf(log, 'Min needle force:        %.4f N\n',   hydraulics.F_needle_N);
    fprintf(log, 'Knee flexion range:      %.2f deg\n', hydraulics.theta_range_deg);
    fprintf(log, 'Cylinder stroke:         %.2f mm\n',   hydraulics.stroke_mm);
    fprintf(log, 'Cylinder min length:     %.2f mm\n',   hydraulics.L_cyl_min_mm);
    fprintf(log, 'Cylinder max length:     %.2f mm\n\n', hydraulics.L_cyl_max_mm);

    % compute frame-by-frame hydraulic pressure series (used for plots 4 & 6)
    pressureSeries = HydraulicAnalysis.getPressureTimeSeries(valve.d_best_mm / 1000);

    % get hydraulic pin dimensions
    fprintf(log, '-------- HYDRAULIC PIN ANALYSIS --------:\n\n');
    pin = HydrPin.PinDim(M_k, FrameAnalysis.w_ball, FrameAnalysis.r_ball, valve.d_best_mm/1000);
    fprintf(log, 'Pin diameter: %.2f mm\n', pin.diameter*1000);
    fprintf(log, 'Upper pin length: %.2f mm\n', pin.length_upper*1000);
    fprintf(log, 'Lower pin length: %.2f mm\n\n', pin.length_lower*1000);

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
    results.d_pin = pin.diameter*1000;
    results.l_pin_upper = pin.length_upper*1000;
    results.l_pin_lower = pin.length_lower*1000;
    results.D_cyl = hydraulics.D_standard_mm;
    results.delta_p_req = hydraulics.delta_p_req_MPa;
    results.Q_max_hyd = hydraulics.Q_max_mLs;
    results.L_restriction = hydraulics.L_restriction_mm;
    results.F_needle = hydraulics.F_needle_N;
    results.stroke_cyl = hydraulics.stroke_mm;
    results.L_cyl_min  = hydraulics.L_cyl_min_mm;
    results.L_cyl_max  = hydraulics.L_cyl_max_mm;

    displayTable = Main.displayResults(results);    % display results to GUI
    results = displayTable;

    % Generate all 6 gait-cycle plots
    GaitPlots.plotAll(gaitSeries, pressureSeries);

    fclose(log);

end

end
end
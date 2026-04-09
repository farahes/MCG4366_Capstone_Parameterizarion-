% =========================================================
% MAIN
% =========================================================
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
%   7. Locking mechanism analysis          — Lock (stresses and spring)
%   8. Pyramid adapter safety factors      — PyramidAdapter (von Mises)

%{

% =========================================================
% SUBSCRIPT CONVENTION
% =========================================================

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

    % 1144 Carbon Steel (https://www.azom.com/article.aspx?ArticleID=6595)
    Sy_s = 620e6; % [Pa] yield strength
    Su_s = 745e6; % [Pa] ultimate strength
    E_s = 190e9; % [Pa] Young's Modulus
    nu_s = 0.27;   % Poisson's ratio

    % FRAME MATERIALS

    % 6061 T6 Aluminum
    Sy_fr = 276e6;    % [Pa] yield strength
    Su_fr = 310e6;    % [Pa] ultimate strength
    E_fr = 68.9e9; % [Pa] Young's Modulus
    nu_fr = 0.33;   % Poisson's ratio
    %{
    % 1010 Carbon Steel
    Sy_fr = 365e6;    % [Pa] yield strength
    Su_fr = 305e6;    % [Pa] ultimate strength
    E_fr = 190e9; % [Pa] Young's Modulus
    nu_fr = 0.27;   % Poisson's ratio
    %}

    % PYRAMID ADAPTER MATERIAL

    % Stainless Steel (none specified, assume AISI 316L )
    Sy_py = 170; % [MPa] yield strength (AISI 316L Stainless Steel)

% =========================================================
% SIZING CHART
% =========================================================
% Sizing is implemented to reduce cost and lead time to manufacture custom
% sizes for each patient

    % STANDARD SHAFT SIZES
    % arranged into a matrix where each row is a size (S/M/L/XL)
    % each row is arranged like: [L_s D_s d_s]
    Shaft_SZ = [0.06 0.022 0.018; 0.0615 0.024 0.020; 0.063 0.026 0.022; 0.063 0.029 0.025]; % dimensions in [m]

    % RETAINGING RING LOCATION
    % from McMaster Carr catalogue for external retaining rings
    % arranged into a matrix where each row is a size (S/M/L/XL)
    % each row is arranged like: [groove_d groove_w]
    Ring_SZ = [0.017 0.0013; 0.019 0.0013; 0.021 0.0013; 0.0239 0.0013]; % dimensions in [m]

    % STANDARD BALL SIZES
    % arranged into a matrix where each row is a size (S/M/L/XL)
    % each row is arranged like: [diameter width]
    Ball_SZ = [0.08 0.06; 0.082 0.0615; 0.084 0.063; 0.095 0.063]; % dimensions in [m]

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

% =========================================================
% DISPLAY FUNCTION
% =========================================================

% Create array of results to display to GUI
function displayTable = displayResults(results)
    % Create table data more explicitly to ensure consistent row counts
    varNames = {
        'D_s'; 'd_s'; 'L_s'; 'l_s'; 'Journal bearing'; 'w_k'; 'r_b'; 'w_b'; 'd_isu'; 't_su';
        'L_fr'; 'a'; 'b'; 't_lp'; 'd_valve'; 'Q'; 'Cv'; 'n_upper_py'; 'n_lower_py';
        'upper_thread_shear'; 'upper_thread_FoS'; 'upper_axial_stress'; 'upper_axial_FoS'; 
        'upper_bending_stress'; 'upper_bending_FoS'; 'upper_vm_stress'; 'upper_vm_FoS';
        'lower_bolt_shear'; 'lower_bolt_FoS'; 'lower_bearing_stress'; 'lower_bearing_FoS';
        'lower_axial_stress'; 'lower_axial_FoS'; 'lower_vm_stress'; 'lower_vm_FoS';
        'lock_k_spring'; 'lock_t_latch'; 'lock_w_key'; 'lock_d_pin';
        'd_pin'; 'l_pin_upper'; 'l_pin_lower'; 'D_cyl'; 'delta_p_req'; 'Q_max_hyd';
        'L_restriction'; 'F_needle'; 'stroke_cyl'; 'L_cyl_min'; 'L_cyl_max'
    };
    
    descriptions = {
        'Large diameter of the shaft'; 'Small diameter of the shaft'; 'Length of large diameter section of the shaft';
        'Total length of the shaft'; 'Journal bearing specifications'; 'Width of the square key';
        'Radius of the ball'; 'Width of the ball'; 'Inner diameter of the supports'; 'Thickness of the supports';
        'Length of the frame legs'; 'Frame leg cross section, long dimension (width)';
        'Frame leg cross section, short dimension (thickness)'; 'Thickness of the lip'; 'Optimal valve diameter';
        'Flow rate'; 'Cv value for valve'; 'Safety factor for upper pyramid adapter';
        'Safety factor for lower pyramid adapter'; 'Upper adapter: Thread shear stress'; 'Upper adapter: Thread stripping FoS';
        'Upper adapter: Axial compression stress'; 'Upper adapter: Axial compression FoS'; 'Upper adapter: Bending stress';
        'Upper adapter: Bending FoS'; 'Upper adapter: Von Mises equivalent stress'; 'Upper adapter: Von Mises FoS';
        'Lower adapter: Bolt shear stress'; 'Lower adapter: Bolt shear FoS'; 'Lower adapter: Bearing stress';
        'Lower adapter: Bearing FoS'; 'Lower adapter: Axial compression stress'; 'Lower adapter: Axial compression FoS';
        'Lower adapter: Von Mises equivalent stress'; 'Lower adapter: Von Mises FoS';
        'Lock spring constant'; 'Lock latch thickness'; 'Lock key width'; 'Lock pin diameter';
        'Hydraulic pin diameter';
        'Upper hydraulic pin length'; 'Lower hydraulic pin length'; 'Hydraulic cylinder bore diameter (standard)';
        'Required cylinder pressure'; 'Maximum hydraulic flow rate'; 'Hagen-Poiseuille restriction channel length';
        'Minimum servo needle axial force'; 'Cylinder piston stroke'; 'Cylinder retracted length at min gait flexion';
        'Cylinder extended length at max gait flexion'
    };
    
    values = {
        round(results.D_s,2); round(results.d_s,2); round(results.L_s,2); round(results.l_s,2);
        sprintf('ID: %.2f, OD: %.2f, Length: %.2f, Part no. %s', results.JBid, results.JBod, results.JBl, results.JBpart);
        round(results.w_k,2); round(results.r_b,2); round(results.w_b,2); round(results.d_isu,2); results.t_su;
        round(results.L_fr,2); results.a; results.b; results.t_lp; results.d_valve; results.Q; results.Cv;
        results.n_upper_py; results.n_lower_py;
        round(results.upper_thread_shear, 2); round(results.upper_thread_FoS, 2); round(results.upper_axial_stress, 2);
        round(results.upper_axial_FoS, 2); round(results.upper_bending_stress, 2); round(results.upper_bending_FoS, 2);
        round(results.upper_vm_stress, 2); round(results.upper_vm_FoS, 2);
        round(results.lower_bolt_shear, 2); round(results.lower_bolt_FoS, 2); round(results.lower_bearing_stress, 2);
        round(results.lower_bearing_FoS, 2); round(results.lower_axial_stress, 2); round(results.lower_axial_FoS, 2);
        round(results.lower_vm_stress, 2); round(results.lower_vm_FoS, 2);
        round(results.lock_k_spring, 2); round(results.lock_t_latch, 2); round(results.lock_w_key, 2); round(results.lock_d_pin, 2);
        round(results.d_pin, 2); round(results.l_pin_upper,2); round(results.l_pin_lower,2); results.D_cyl;
        results.delta_p_req; results.Q_max_hyd; results.L_restriction; results.F_needle; results.stroke_cyl;
        results.L_cyl_min; results.L_cyl_max
    };
    
    units = {
        'mm'; 'mm'; 'mm'; 'mm'; 'mm (diameters and length)'; 'mm'; 'mm'; 'mm'; 'mm'; 'mm';
        'mm'; 'mm'; 'mm'; 'mm'; 'mm'; 'L/min'; 'N/A'; 'N/A'; 'N/A';
        'MPa'; 'N/A'; 'MPa'; 'N/A'; 'MPa'; 'N/A'; 'MPa'; 'N/A';
        'MPa'; 'N/A'; 'MPa'; 'N/A'; 'MPa'; 'N/A'; 'MPa'; 'N/A';
        'N/m'; 'mm'; 'mm'; 'mm';
        'mm'; 'mm'; 'mm'; 'mm'; 'MPa'; 'mL/s'; 'mm'; 'N'; 'mm'; 'mm'; 'mm'
    };
    
    % Verify all arrays have same length
    n = length(varNames);
    assert(length(descriptions) == n, 'Descriptions array length mismatch');
    assert(length(values) == n, 'Values array length mismatch');
    assert(length(units) == n, 'Units array length mismatch');
    
    % Create table
    displayTable = table(varNames, descriptions, values, units, 'VariableNames', {'Variable Name','Description','Value','Units'});

end

% =========================================================
% SOLIDWORKS INTEGRATION
% =========================================================

% export results to text files for SolidWorks integration
function exportDimensions(dimensions)
    
    basePath = fileparts(mfilename('fullpath'));
    eqDir = fullfile(basePath, 'Solidworks', 'Equations');
    if ~isfolder(eqDir)
        [ok, msg] = mkdir(eqDir);
        if ~ok
            error('Could not create Solidworks/Equations directory: %s', msg);
        end
    end

    %-------- SHAFT DIMENSIONS --------
    shaft_filePath = fullfile(eqDir, 'shaft.txt');
    shaft = fopen(shaft_filePath, 'w');
    if shaft == -1
        error('Could not create shaft.txt');
    end

    fprintf(shaft, '"large_shaft_diameter"=%.2f\n',dimensions.D_s);
    fprintf(shaft, '"small_shaft_diameter"=%.2f\n',dimensions.d_s);
    fprintf(shaft, '"length_shaft"=%.2f\n',dimensions.l_s);
    fprintf(shaft, '"large_length_shaft"=%.2f\n',dimensions.L_s/2); % divided by two bc that's how the SW part is made
    fprintf(shaft, '"width_key"=%.2f\n',dimensions.w_k + 0.5); % key width + 0.5mm tolerance
    fprintf(shaft, '"snap_ring_location"=%.2f\n',dimensions.L_s/2 + 1.00 + 0.25 + dimensions.JBl);
    fprintf(shaft, '"snap_ring_groove_radius"=%.2f\n',dimensions.rr_r);
    fprintf(shaft, '"snap_ring_groove_width"=%.2f\n',dimensions.rr_w);
    fclose(shaft);

    %-------- JOURNAL BEARING DIMENSIONS --------
    JB_filePath = fullfile(eqDir, 'journalbearing.txt');
    JB = fopen(JB_filePath, 'w');
    if JB == -1
        error('Could not create journalbearing.txt');
    end

    fprintf(JB, '"inner_diameter"=%.2f\n', dimensions.JBid);
    fprintf(JB, '"outer_diameter"=%.2f\n', dimensions.JBod);
    fprintf(JB, '"length"=%.2f\n', dimensions.JBl);
    fprintf(JB, '"flange_thickness"=%.2f\n', dimensions.JBft);
    fprintf(JB, '"flange_diameter"=%.2f\n', dimensions.JBfod);
    fclose(JB);

    %-------- KEY DIMENSIONS --------
    key_filePath = fullfile(eqDir, 'key.txt');
    key = fopen(key_filePath, 'w');
    if key == -1
        error('Could not create key.txt');
    end

    fprintf(key, '"width_key"=%.2f\n',dimensions.w_k);
    fprintf(key, '"length_key"=%.2f\n',dimensions.L_s - 0.5);
    fclose(key);

    %-------- BALL DIMENSIONS --------
    ball_filePath = fullfile(eqDir, 'ball.txt');
    ball = fopen(ball_filePath, 'w');
    if ball == -1
        error('Could not create ball.txt');
    end

    fprintf(ball, '"width_ball"=%.2f\n',dimensions.w_b);
    fprintf(ball, '"diameter_ball"=%.2f\n',dimensions.r_b*2);
    fprintf(ball, '"lip_ball"=%.2f\n',dimensions.r_b + dimensions.t_lp);
    fprintf(ball, '"shaft_bore"=%.2f\n',dimensions.D_s + 0.5);
    fprintf(ball, '"key_slot"=%.2f\n',dimensions.w_k + 0.5);
    fprintf(ball, '"shaft_extrude"=%.2f\n',dimensions.D_s + 15);
    fprintf(ball, '"adapter_slice"=%.2f\n',dimensions.r_b*0.8125);
    fprintf(ball, '"hydraulic_pin_bore"=%.2f\n',dimensions.d_pin + 0.5);
    fprintf(ball, '"hydraulic_pin_radius"=%.2f\n',dimensions.r_b*0.75);
    fprintf(ball, '"hydraulic_slot_width"=%.2f\n',(dimensions.D_cyl + 4)/2);    % divided by two since that's how it's defined in SW
    fprintf(ball, '"hydraulic_slot_diameter"=%.2f\n',dimensions.d_pin*1.65);
    fprintf(ball, '"hydraulic_slot_half_diameter"=%.2f\n',(dimensions.d_pin*1.65 + (dimensions.d_pin + 0.5))/2);
    fclose(ball);

    %-------- FRAME DIMENSIONS --------
    frame_filePath = fullfile(eqDir, 'frame.txt');
    frame = fopen(frame_filePath, 'w');
    if frame == -1
        error('Could not create frame.txt');
    end

    fprintf(frame, '"outer_diameter"=%.2f\n',2*dimensions.r_b + 2*dimensions.t_lp + 0.5);
    fprintf(frame, '"outer_width"=%.2f\n',dimensions.w_b + 2*dimensions.t_su + 2*1.00 + 0.5);
    fprintf(frame, '"leg_length"=%.2f\n',dimensions.L_fr);
    fprintf(frame, '"inner_diameter"=%.2f\n',2*dimensions.r_b + 0.5);
    fprintf(frame, '"support_thickness"=%.2f\n',dimensions.t_su);
    fprintf(frame, '"shaft_extrude_diameter"=%.2f\n',dimensions.JBod + 15);
    fprintf(frame, '"shaft_extrude_thickness"=%.2f\n',dimensions.JBl - dimensions.JBft - dimensions.t_su);
    fprintf(frame, '"shaft_bore"=%.2f\n',dimensions.JBod);
    fprintf(frame, '"support_diameter"=%.2f\n',dimensions.JBod/2 + 15);
    fprintf(frame, '"leg_thickness"=%.2f\n',dimensions.b);
    fprintf(frame, '"leg_width"=%.2f\n',dimensions.a);
    fprintf(frame, '"hydraulic_pin_bore"=%.2f\n',dimensions.d_pin);
    fclose(frame);

    %{
    %-------- HYDRAULIC PIN DIMENSIONS - TOP --------
    hydrpin_top_filePath = fullfile(basePath,'..', 'Solidworks', 'Equations', 'hydrpin.txt');
    hydrpin = fopen(hydrpin_filePath, 'w');
    if hydrpin == -1
        error('Could not create hydrpin.txt');
    end

    fprintf(hydrpin, '"diameter"=%.2f\n',);
    fprintf(hydrpin, '"length"=%.2f\n',);
    fprintf(hydrpin, '"handle_keys"=%.2f\n',);
    fprintf(hydrpin, '"latch_keys_width"=%.2f\n',);
    fclose(hydrpin);

    %-------- HYDRAULIC PIN DIMENSIONS - BOTTOM --------
    hydrpin_top_filePath = fullfile(basePath,'..', 'Solidworks', 'Equations', 'hydrpin.txt');
    hydrpin = fopen(hydrpin_filePath, 'w');
    if hydrpin == -1
        error('Could not create hydrpin.txt');
    end

    fprintf(hydrpin, '"diameter"=%.2f\n',);
    fprintf(hydrpin, '"length"=%.2f\n',);
    fclose(hydrpin);

    %-------- LOCK DIMENSIONS - PIN --------
    lock_pin_filePath = fullfile(basePath,'..', 'Solidworks', 'Equations', 'lock_pin.txt');
    lock_pin = fopen(lock_pin_filePath, 'w');
    if lock_pin == -1
        error('Could not create lock_pin.txt');
    end

    fprintf(lock_pin, '"diameter"=%.2f\n',);
    fprintf(lock_pin, '"length"=%.2f\n',);
    fprintf(lock_pin, '"handle_keys"=%.2f\n',);
    fprintf(lock_pin, '"latch_keys_width"=%.2f\n',);
    fclose(lock_pin);

    %-------- LOCK DIMENSIONS - HANDLE --------
    lock_handle_filePath = fullfile(basePath,'..', 'Solidworks', 'Equations', 'lock_handle.txt');
    lock_handle = fopen(lock_handle_filePath, 'w');
    if lock_handle == -1
        error('Could not create lock_handle.txt');
    end

    fprintf(lock_handle, '"length"=%.2f\n',);
    fprintf(lock_handle, '"thickness"=%.2f\n',);
    fprintf(lock_handle, '"bore_diameter"=%.2f\n',);
    fprintf(lock_handle, '"handle_keys"=%.2f\n',);
    fclose(lock_handle);

    %-------- LOCK DIMENSIONS - LATCH --------
    lock_latch_filePath = fullfile(basePath,'..', 'Solidworks', 'Equations', 'lock_latch.txt');
    lock_latch = fopen(lock_latch_filePath, 'w');
    if lock_latch == -1
        error('Could not create lock_latch.txt');
    end

    fprintf(lock_latch, '"length"=%.2f\n',);
    fprintf(lock_latch, '"width"=%.2f\n',);
    fprintf(lock_latch, '"thickness"=%.2f\n',);
    fprintf(lock_latch, '"slot_diameter"=%.2f\n',);
    fprintf(lock_latch, '"slot_length"=%.2f\n',);
    fprintf(lock_latch, '"latch_keys_width"=%.2f\n',);
    fprintf(lock_latch, '"latch_keys_length"=%.2f\n',);
    fprintf(lock_latch, '"spring_cavity_diameter"=%.2f\n',);
    fclose(lock_latch);

    %-------- LOCK DIMENSIONS - SPRING --------

    %}
end

% /***************************************/
% /                 MAIN                  /
% /***************************************/

function results = getResults(BW, H)

    % Add subfolders to MATLAB path so class files can be found
    rootDir = fileparts(which('Main'));
    addpath(fullfile(rootDir, 'analysis'));
    addpath(fullfile(rootDir, 'components'));

    % create log file in the project logs folder
    logDir = fullfile(rootDir, 'logs');
    if ~isfolder(logDir)
        [ok, msg] = mkdir(logDir);
        if ~ok
            error('Could not create log directory: %s', msg);
        end
    end
    log_filePath = fullfile(logDir, 'group12_LOG.txt');
    log = fopen(log_filePath, 'w');
    if log == -1
        error('Could not create log file');
    end
    fprintf(log, '/***************************************/\n');
    fprintf(log, '/               ANALYSIS LOG            /\n');
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

    sz = 1; % index representing the row in the sizing matrix (start with the smallest size)

    % iterate until shaft length and diameter converge to the same size
    while (true)
        fprintf(log, 'Size %i:\n', sz);
        % get limiting shaft dimension
        rawDiameter = ShaftAnalysis.getShaftDiameter( ...
            log, ...
            Main.n_shaft, ...
            Main.Su_s, ...
            Main.Sy_s, ...
            Main.getSsy(Main.Sy_s), ...
            F_kx, ...
            F_ky, ...
            M_k, ...
            Main.Shaft_SZ(sz,1) ...
            );

        % get journal bearing specs
        fprintf(log, 'Journal bearing for inner diameter of %.2fmm:\n', rawDiameter.d*1000);
        JB = ShaftAnalysis.getJB(F_k, rawDiameter.d);
        fprintf(log, 'ID: %.2fmm\n', JB.id*1000);
        fprintf(log, 'OD: %.2fmm\n', JB.od*1000);
        fprintf(log, 'Length: %.2fmm\n', JB.l*1000);
        fprintf(log, 'Flange thickness: %.2fmm\n', JB.ft*1000);
        fprintf(log, 'Flange OD: %.2fmm\n', JB.fod*1000);
        fprintf(log, 'Part no. %s\n\n', JB.part{1});
    
        % get adjusted shaft dimensions from journal bearing
        adjDiameter = ShaftAnalysis.getFinalShaftDimensions(JB, Main.Shaft_SZ(sz,1));
        fprintf(log, 'Adjusted shaft dimensions after journal bearing selection:\n');
        d_s = adjDiameter.d_s;
        D_s = adjDiameter.d_s + 0.004;  % Large diameter will be 4mm larger than the small diameter
        fprintf(log, 'D_s: %.2f mm\n', D_s*1000);
        fprintf(log, 'd_s: %.2f mm\n', d_s*1000);
        fprintf(log, 'L_s: %.2f mm\n', Main.Shaft_SZ(sz,1)*1000);
        fprintf(log, 'l_s: %.2f mm\n\n', adjDiameter.l_s*1000);
    
        % check torsional deflection
        while ~ShaftAnalysis.getCheckTorsion(log, M_k, Main.getG(Main.E_s, Main.nu_s), Main.Shaft_SZ(sz,1), D_s, adjDiameter.l_s-Main.Shaft_SZ(sz,1), d_s)
            D_s = D_s + 0.001;
            d_s = d_s + 0.001;
            JB = ShaftAnalysis.getJB(F_k, d_s);
            adjDiameter = ShaftAnalysis.getFinalShaftDimensions(JB);
        end
   
        % check if diameter size is larger than the length size
        if (d_s > Main.Shaft_SZ(sz,3)) || (D_s > Main.Shaft_SZ(sz,2))
            fprintf(log, 'Shaft diameter and length sizes do not match.\n\n');
            sz = sz + 1;
        else
            break;
        end
        
    end

    fprintf(log, 'Shaft diameter and length sizes converged!\n\n');

    fprintf(log, 'Final adjusted shaft dimensions:\n');
    fprintf(log, 'D_s: %.2f mm\n', Main.Shaft_SZ(sz,2)*1000);
    fprintf(log, 'd_s: %.2f mm\n', Main.Shaft_SZ(sz,3)*1000);
    fprintf(log, 'L_s: %.2f mm\n', Main.Shaft_SZ(sz,1)*1000);
    fprintf(log, 'l_s: %.2f mm\n\n', adjDiameter.l_s*1000);
    

    % get key dimension
    w_k = ShaftAnalysis.getWk(Main.Shaft_SZ(sz,2));
    fprintf(log, 'Width of key: %.2f mm\n\n', w_k*1000);

    % get retaining ring groove dimensions
    rr_r = Main.Ring_SZ(sz,1)/2;  % [m], groove radius
    rr_w = Main.Ring_SZ(sz,2);  % [m], groove width

    % export retaining ring size
    Rings.getRRing(Main.Shaft_SZ(sz,3)*1000);

    fprintf(log, '-------- SELECT BALL SIZE FOR CORRESPONDING SHAFT SIZE --------:\n\n');

    r_b = Main.Ball_SZ(sz,1)/2; % [m], radius of the ball
    w_b = Main.Ball_SZ(sz,2);   % [m], width of the ball

    fprintf(log, 'Ball radius: %.2f mm\n', r_b*1000);
    fprintf(log, 'Ball width: %.2f mm\n\n', w_b*1000);

    fprintf(log, '-------- FRAME ANALYSIS --------:\n\n');

    % get support thickness
    t_s = FrameAnalysis.getSupportThickness(log, Main.n_frame, F_k, Main.E_fr, Main.getSsy(Main.Sy_fr), Main.Sy_fr, JB.od, r_b);

    % get leg cross-section dimensions
    legDimensions = FrameAnalysis.getLegDimensions(log, Main.n_frame, F_k, Main.E_fr, Main.Sy_fr, Main.getSsy(Main.Sy_fr), JB.od, r_b);

    % get lip thickness
    F_lp = FrameAnalysis.getFlp(M_k, F_k, r_b);
    t_lp = FrameAnalysis.getLipThicknessComp(log, F_lp, FrameAnalysis.getSigma(Main.Sy_fr, Main.n_frame), w_b);

    % get final frame thickness
    t_frame = FrameAnalysis.t_frame(log, t_s, legDimensions.b, t_lp);

    % get needle valve calculations and plot
    valve = HydraulicNeedleValve.getValveSize();
    % HydraulicNeedleValve.plot(valve); TURNED OFF, NO CHANGE IN PLOT

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
    pin = HydrPin.PinDim(M_k, w_b, r_b, valve.d_best_mm/1000);
    fprintf(log, 'Pin diameter: %.2f mm\n', pin.diameter*1000);
    fprintf(log, 'Upper pin length: %.2f mm\n', pin.length_upper*1000);
    fprintf(log, 'Lower pin length: %.2f mm\n\n', pin.length_lower*1000);

    % get lock dimensions and integrate results into GUI output
    fprintf(log, '-------- LOCK ANALYSIS --------:\n\n');
    m_f = JointReactionForce.getMf(BW);
    m_ll = JointReactionForce.getMl(BW);
    d_f = JointReactionForce.getLl(H) + 0.5*JointReactionForce.getLf(H);
    d_ll = 0.433*JointReactionForce.getLl(H);
    lock_dimensions = Lock.LockDim(log, m_f, m_ll, d_f, d_ll, M_k, w_b);

    % get pyramid adapter safety factors
    fprintf(log, '-------- PYRAMID ADAPTER ANALYSIS --------:\n\n');
    
    % Upper Adapter Analysis
    [tau_s_upper, n_strip_upper, thread_pass] = PyramidAdapter.analyzeThreadStripping();
    [sigma_ax_upper, n_ax_upper] = PyramidAdapter.analyzeUpperAxialCompression();
    [sigma_bend_upper, n_bend_upper] = PyramidAdapter.analyzeUpperBending();
    [~, ~, sigma_vm_upper, n_vm_upper] = PyramidAdapter.analyzeUpperCombinedStress();
    
    % Lower Adapter Analysis
    [tau_bolt_lower, n_bolt_lower, bolt_pass] = PyramidAdapter.analyzeBoltShear();
    [sigma_bear_lower, n_bear_lower, bear_pass] = PyramidAdapter.analyzeBoltBearing();
    [sigma_ax_lower, n_ax_lower] = PyramidAdapter.analyzeLowerAxialCompression();
    [~, ~, sigma_vm_lower, n_vm_lower] = PyramidAdapter.analyzeLowerCombinedStress();
    
    % Compatibility functions for original FoS calculation
    FoS_yield_u = PyramidAdapter.getFoS_yield_u(F_kx, F_ky, Main.Sy_py);
    FoS_yield_b = PyramidAdapter.getFoS_yield_b(F_kx, F_ky, Main.Sy_py);
    
    % Log detailed pyramid adapter results
    fprintf(log, '--- UPPER ADAPTER ---\n');
    fprintf(log, 'Thread Stripping: Shear Stress = %.2f MPa, FoS = %.2f, Pass = %d\n', tau_s_upper, n_strip_upper, thread_pass);
    fprintf(log, 'Axial Compression: Stress = %.2f MPa, FoS = %.2f\n', sigma_ax_upper, n_ax_upper);
    fprintf(log, 'Bending: Stress = %.2f MPa, FoS = %.2f\n', sigma_bend_upper, n_bend_upper);
    fprintf(log, 'Combined (Von Mises): Stress = %.2f MPa, FoS = %.2f\n\n', sigma_vm_upper, n_vm_upper);
    
    fprintf(log, '--- LOWER ADAPTER ---\n');
    fprintf(log, 'Bolt Shear: Stress = %.2f MPa, FoS = %.2f, Pass = %d\n', tau_bolt_lower, n_bolt_lower, bolt_pass);
    fprintf(log, 'Bolt Bearing: Stress = %.2f MPa, FoS = %.2f, Pass = %d\n', sigma_bear_lower, n_bear_lower, bear_pass);
    fprintf(log, 'Axial Compression: Stress = %.2f MPa, FoS = %.2f\n', sigma_ax_lower, n_ax_lower);
    fprintf(log, 'Combined (Von Mises): Stress = %.2f MPa, FoS = %.2f\n\n', sigma_vm_lower, n_vm_lower);

    results.D_s = Main.Shaft_SZ(sz,2)*1000;
    results.d_s = Main.Shaft_SZ(sz,3)*1000;
    results.L_s = Main.Shaft_SZ(sz,1)*1000;
    results.l_s = adjDiameter.l_s*1000;
    results.JBid = JB.id*1000;
    results.JBod = JB.od*1000;
    results.JBl = JB.l*1000;
    results.JBft = JB.ft*1000;
    results.JBfod = JB.fod*1000;
    results.JBpart = JB.part{1};
    results.rr_r = rr_r*1000;
    results.rr_w = rr_w*1000;
    results.w_k = w_k*1000;
    results.r_b = r_b*1000;
    results.w_b = w_b*1000;
    results.d_isu = JB.od*1000;
    results.t_su = t_frame*1000;
    results.L_fr = FrameAnalysis.L_fr*1000;
    results.a = legDimensions.a*1000;
    results.b = t_frame*1000;
    results.t_lp = t_frame*1000;
    results.d_valve = valve.d_best_mm;
    results.Q = valve.Q_Lmin;
    results.Cv = valve.Cv;
    results.n_upper_py = FoS_yield_u;
    results.n_lower_py = FoS_yield_b;
    
    % Upper Adapter Detailed Results
    results.upper_thread_shear = tau_s_upper;
    results.upper_thread_FoS = n_strip_upper;
    results.upper_axial_stress = sigma_ax_upper;
    results.upper_axial_FoS = n_ax_upper;
    results.upper_bending_stress = sigma_bend_upper;
    results.upper_bending_FoS = n_bend_upper;
    results.upper_vm_stress = sigma_vm_upper;
    results.upper_vm_FoS = n_vm_upper;
    
    % Lower Adapter Detailed Results
    results.lower_bolt_shear = tau_bolt_lower;
    results.lower_bolt_FoS = n_bolt_lower;
    results.lower_bearing_stress = sigma_bear_lower;
    results.lower_bearing_FoS = n_bear_lower;
    results.lower_axial_stress = sigma_ax_lower;
    results.lower_axial_FoS = n_ax_lower;
    results.lower_vm_stress = sigma_vm_lower;
    results.lower_vm_FoS = n_vm_lower;

    % Lock Mechanism Detailed Results
    results.lock_k_spring = lock_dimensions.k;
    results.lock_t_latch = lock_dimensions.t_latch*1000;
    results.lock_w_key = lock_dimensions.w_key*1000;
    results.lock_d_pin = lock_dimensions.d_pin*1000;

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

    dimensions = results;   % save results before converting to struct
    Main.exportDimensions(dimensions); % export dimensions for SolidWorks

    % Generate all 6 gait-cycle plots in separate figure windows
    % GaitPlots.plotAll(gaitSeries, pressureSeries); TURNED OFF

    fclose(log);

    % Return struct so GUI can also render plots directly into uiaxes
    results                = struct();
    results.table          = displayTable;
    results.gaitSeries     = gaitSeries;
    results.pressureSeries = pressureSeries;

end

end
end
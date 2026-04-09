% =========================================================
% HYDRAULIC UNIT ANALYSIS
% MCG4366 Group 12
%
% Sizes all components of the hydraulic damping unit:
%   1.  Maximum axial load on the cylinder             (Eq. 65)
%   2.  Maximum knee angular velocity & piston vel     (Eq. 66)
%   3.  Minimum cylinder bore diameter                 (Eq. 67)
%   4.  Required pressure drop verification            (Eq. 68)
%   5.  Maximum volumetric flow rate                   (Eq. 69)
%   6.  Silicone oil density at 30 °C                 (thermal expansion)
%   7.  Silicone oil dynamic viscosity at 30 °C       (Eq. 70)
%   8.  Hagen-Poiseuille restriction length            (Eq. 71)
%   9.  Minimum axial needle force (servo sizing)
%  10.  Knee flexion range from Winter's gait data
%  11.  Required cylinder piston stroke
%
% All derivations follow Section 5.1 and Appendix C/E of the design report.
% =========================================================

classdef HydraulicAnalysis

properties (Constant)

    % ---- USER & GEOMETRY ----
    m_user      = 70;           % Maximum user mass [kg]  (Table 13)
    g           = 9.81;         % Gravitational acceleration [m/s^2]
    r_ball      = 0.04;         % Lever arm: knee axis to damper line of action [m]  (Table 13)

    % ---- KINEMATICS ----
    % Maximum knee angular velocity during gait at ~1.0-1.19 m/s walking speed.
    % Source: Winter's gait data (ref [33] in working analysis report).
    theta_dot_max_degs = 357.90; % [deg/s]

    % ---- CYLINDER ----
    % delta_p_allow chosen as 8 MPa: conservative upper bound within the
    % 3-10 MPa range reported for hydraulic prosthetic knees (refs [34][35][36] in working analysis report).
    delta_p_allow = 8e6;        % Maximum allowable cylinder pressure [Pa]
    D_standard    = 0.020;      % Selected standard cylinder bore: 20 mm (rounded up from D_min)

    % ---- SILICONE OIL (Polydimethylsiloxane, 350 cSt grade, ref [37] in working analysis report) ----
    % 350 cSt selected as medium-viscosity grade balancing damping performance,
    % temperature sensitivity, and seal durability (300-500 cSt range, Appendix E).
    rho_25  = 970;              % Fluid density at 25 °C [kg/m^3]
    nu_s    = 350e-6;           % Kinematic viscosity at 30 °C [m^2/s]  (nu_30 ≈ nu_25 assumed)
    beta    = 0.8e-3;           % Volumetric thermal expansion coefficient [1/°C]
    delta_T = 5;                % Temperature difference: 30 °C - 25 °C = 5 °C

end

methods (Static)

% =========================================================
%  STEP 1 — MAXIMUM AXIAL LOAD ON THE CYLINDER
% =========================================================
% The peak ground reaction force (GRF) during normal walking reaches
% approximately 1.2 BW (body weight). For prosthetic design a factor of
% safety of ≈ 2.5 is applied, giving a governing load of 3× BW:
%
%   F_load,max = 3 × m_user × g
%              = 3 × 70  × 9.81
%              ≈ 2060 N
%
% This 3 BW load is the standard worst-case design load for lower-limb
% prostheses (ISO 10328) and governs bore diameter and pressure sizing.
function F_load = getMaxLoad()
    F_load = 3 * HydraulicAnalysis.m_user * HydraulicAnalysis.g;  % [N]
end

% =========================================================
%  STEP 2 — MAXIMUM ANGULAR AND PISTON VELOCITY
% =========================================================
% Step 2a — Convert angular velocity to rad/s:
%   omega_max = theta_dot_max_degs × (pi/180)
%
% Step 2b — Piston tip speed via arc-length formula:
%   If a point on a rigid body is at radius r from the pivot:
%     ds/dt = r × d(theta)/dt
%     v_p,max = r_ball × omega_max
%
%   where r_ball is the perpendicular distance from the knee joint
%   axis to the cylinder attachment point (the moment arm).
%
% The worst case occurs during terminal swing, when the knee is
% extending rapidly and the valve is fully open (no restriction).
function vel = getMaxVelocity()
    omega_max = HydraulicAnalysis.theta_dot_max_degs * (pi/180);  % [rad/s]
    v_p_max   = HydraulicAnalysis.r_ball * omega_max;             % [m/s]
    vel.omega_max = omega_max;
    vel.v_p_max   = v_p_max;
end

% =========================================================
%  STEP 3 — MINIMUM CYLINDER BORE DIAMETER
% =========================================================
% Force-pressure-area relationship for a hydraulic cylinder:
%   F = P × A_p     →    A_p = F / P
%
% Piston face area in terms of bore diameter D:
%   A_p = pi × D^2 / 4
%
% Setting A_p = F_load,max / delta_p_allow and solving for D:
%   pi × D_min^2 / 4 = F_load,max / delta_p_allow
%   D_min = sqrt( 4 × F_load,max / (pi × delta_p_allow) )
%         ≈ 18.1 mm
%
% The nearest standard catalogue bore above 18.1 mm is 20 mm,
% so D_standard = 20 mm is selected.
function D_min = getMinCylinderDiameter()
    F_load = HydraulicAnalysis.getMaxLoad();
    D_min  = sqrt(4 * F_load / (pi * HydraulicAnalysis.delta_p_allow));  % [m]
end

% =========================================================
%  STEP 4 — VERIFY REQUIRED PRESSURE AT STANDARD BORE
% =========================================================
% Now that D_standard = 20 mm is chosen, recalculate the actual
% operating pressure required to support F_load,max:
%
%   A_20mm  = pi × D_standard^2 / 4   (piston face area)
%   delta_p_req = F_load,max / A_20mm
%
% D_standard = 20 mm > D_min = 18.1 mm, so A_20mm > A_min,
% and therefore delta_p_req < delta_p_allow:  this confirms the
% selected bore gives an operating pressure within the 5-10 MPa
% design range for pneumatic/hydraulic prosthetic knees.
function delta_p = getRequiredPressure()
    F_load  = HydraulicAnalysis.getMaxLoad();
    A_20mm  = pi * HydraulicAnalysis.D_standard^2 / 4;  % [m^2]
    delta_p = F_load / A_20mm;                           % [Pa]
end

% =========================================================
%  STEP 5 — MAXIMUM VOLUMETRIC FLOW RATE
% =========================================================
% From the continuity equation for incompressible flow,
% volumetric flow rate through the piston is:
%
%   Q = A_p × v_p
%
% At maximum piston velocity v_p,max (from Step 2) and with
% the standard 20 mm bore piston area A_p:
%
%   Q_max = (pi × D_standard^2 / 4) × v_p,max
%
% This maximum flow rate is the design value for:
%   - Needle valve orifice diameter selection (Step 8, HydraulicNeedleValve)
%   - Hagen-Poiseuille restriction channel length (Step 8 here)
function Q = getMaxFlowRate()
    A_p = pi * HydraulicAnalysis.D_standard^2 / 4;  % [m^2]
    vel = HydraulicAnalysis.getMaxVelocity();
    Q   = A_p * vel.v_p_max;                         % [m^3/s]
end

% =========================================================
%  STEP 6 — SILICONE OIL DENSITY AT 30 °C
% =========================================================
% Thermal volumetric expansion of a liquid:
%   V(T) = V_25 × (1 + beta × (T - 25))       (linear approximation)
%
% where beta = volumetric thermal expansion coefficient [1/°C].
%
% Since mass is conserved (m = rho × V = constant):
%   rho(T) = m / V(T) = m / (V_25 × (1 + beta × delta_T))
%          = rho_25 / (1 + beta × delta_T)
%
% With delta_T = 30 - 25 = 5 °C and beta = 0.8×10^-3 /°C:
%   rho_30 = 970 / (1 + 0.8e-3 × 5) ≈ 966 kg/m³
%
% The 30 °C operating temperature accounts for heat generated
% by viscous dissipation during continuous damping.
function rho_30 = getFluidDensity30()
    rho_30 = HydraulicAnalysis.rho_25 / ...
             (1 + HydraulicAnalysis.beta * HydraulicAnalysis.delta_T);  % [kg/m^3]
end

% =========================================================
%  STEP 7 — DYNAMIC VISCOSITY AT 30 °C
% =========================================================
% By definition, kinematic viscosity relates to dynamic viscosity by:
%   nu = mu / rho    →    mu = nu × rho
%
% The silicone oil grade is specified as 350 cSt at 25 °C.
% nu_30 ≈ nu_25 = 350×10^-6 m²/s is assumed because:
%   - The temperature increase is only 5 °C
%   - PDMS viscosity is relatively temperature-stable in this range
%
% Using rho_30 from Step 6:
%   mu = nu_s × rho_30 = 350e-6 × 966 ≈ 0.338 Pa·s
%
% mu is required by Hagen-Poiseuille in Step 8.
function mu = getDynamicViscosity()
    rho_30 = HydraulicAnalysis.getFluidDensity30();
    mu     = HydraulicAnalysis.nu_s * rho_30;  % [Pa·s]
end

% =========================================================
%  STEP 8 — HAGEN-POISEUILLE RESTRICTION CHANNEL LENGTH
% =========================================================
% Full Hagen-Poiseuille derivation (Appendix C):
%   Velocity profile: u(r) = (-1/(4*mu)) * (dp/dz) * (R^2 - r^2)
%   Integrating over the cross-section: Q = -pi*R^4/(8*mu) * dp/dz
%   Substituting dp/dz = -delta_p/L and R = d/2, solved for L:
%
%   L = (pi * d_eff^4 * delta_p_allow) / (128 * mu * Q_max)  
%
% Assumptions (Re_D ≤ 2100): laminar, steady, incompressible,
% fully-developed, axisymmetric flow in a circular channel.
%
% Input: d_eff [m] — effective needle valve orifice diameter
function L = getRestrictionLength(d_eff)
    mu    = HydraulicAnalysis.getDynamicViscosity();
    Q_max = HydraulicAnalysis.getMaxFlowRate();
    L     = (pi * d_eff^4 * HydraulicAnalysis.delta_p_allow) / (128 * mu * Q_max);  % [m]
end

% =========================================================
%  STEP 9 — MINIMUM AXIAL NEEDLE FORCE FOR SERVO SIZING
% =========================================================
% The hydraulic pressure difference across the needle valve exerts a
% net force on the needle face that the servo motor must overcome
% to open or close the restriction.
%
% For any surface with area A at uniform pressure P:
%   F = integral of P dA = P × A      (pressure is uniform over A)
%
% The relevant area is the circular seat face exposed to the full
% pressure differential (the orifice cross-section):
%   A_seat = pi × d_eff^2 / 4
%
% Therefore the minimum axial force the servo arm must supply:
%   F_axial,min = delta_p_allow × A_seat
%              = delta_p_allow × pi × d_eff^2 / 4
%
% Simplifications applied:
%   - Pressure is uniform across the small seat face (d_eff << pipe D)
%   - Viscous shear on the needle tip perimeter is negligible compared
%     to the pressure-area term at 8 MPa
%   - Only the axial component is considered (needle travels axially)
%
% This force is the minimum torque requirement for the MG90S servo.
%
% Input: d_eff [m] — effective needle valve orifice diameter
function F_axial = getNeedleForce(d_eff)
    A_seat  = pi * d_eff^2 / 4;
    F_axial = HydraulicAnalysis.delta_p_allow * A_seat;  % [N]
end

% =========================================================
%  STEP 10 — KNEE JOINT ANGLE EXTREMES FROM GAIT DATA
% =========================================================
% In Winter's data each CSV stores SEGMENT absolute angles measured
% counter-clockwise from the right horizontal (+x = forward, +y = up):
%
%   WinterAppendix_KinKnee.csv  Theta column = THIGH segment angle
%   WinterAppendix_KinLeg.csv   Theta column = SHANK segment angle
%
% The knee JOINT angle at each frame is the difference:
%   theta_joint = theta_shank − theta_thigh   (Winter sign convention)
%
% Verification against known gait data:
%   Frame 1 (heel strike):    joint ≈ 55°  → ~5° anatomical flexion  ✓
%   Peak swing (~65% cycle):  joint ≈ 112° → ~62° anatomical flexion ✓
%   Range ≈ 62°  — consistent with ~65° peak flexion at 1.0–1.19 m/s ✓
%
% NOTE: The previous implementation read only the thigh column from
% KinKnee, giving 27.4° — underestimating stroke by ~55%.  Fixed here.
function extremes = getJointAngleExtremes()
    rootDir    = fileparts(fileparts(which('HydraulicAnalysis')));
    kneeTable  = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinKnee.csv'));
    legTable   = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinLeg.csv'));
    thigh_theta = kneeTable.(matlab.lang.makeValidName('Theta (deg)'));   % thigh segment [deg]
    shank_theta = legTable.(matlab.lang.makeValidName('Theta (deg)'));    % shank segment [deg]
    joint_theta = shank_theta - thigh_theta;                             % joint angle at each frame
    extremes.theta_min_deg = min(joint_theta);  % most extended in gait
    extremes.theta_max_deg = max(joint_theta);  % most flexed in gait (peak swing)
end

% =========================================================
%  STEP 10b — KNEE FLEXION RANGE
% =========================================================
% Returns the total angular excursion of the knee joint over the full
% gait cycle.  The cylinder must stroke through this entire range.
function theta_range_deg = getKneeFlexionRange()
    extremes = HydraulicAnalysis.getJointAngleExtremes();
    theta_range_deg = extremes.theta_max_deg - extremes.theta_min_deg;
end

% =========================================================
%  STEP 11 — CYLINDER PISTON STROKE
% =========================================================
% The cylinder is mounted posteriorly (behind the knee) with:
%   - One pivot fixed to the frame at distance r_ball from the knee axis
%   - The other pivot at the piston rod end (free to move)
%
% As the knee rotates by angle theta_range, the fixed-pivot end
% of the cylinder traces an arc of radius r_ball. The linear
% displacement of the piston (stroke) equals this arc length:
%
%   arc length:  s = r × theta   (in radians)
%   stroke = r_ball × theta_range_rad
%          = r_ball × (theta_range_deg × pi/180)
%
% Accuracy note:
%   The arc-length equals the chord length exactly only for theta→0.
%   For the true chord: chord = 2 × r_ball × sin(theta_range_rad / 2)
%   At theta_range ≈ 67°:  chord ≈ r_ball × 1.134,  arc ≈ r_ball × 1.169
%   The arc-length overestimates stroke by ~3%, giving a conservative
%   (slightly larger) cylinder selection — acceptable for design sizing.
function stroke = getStroke()
    theta_range_rad = deg2rad(HydraulicAnalysis.getKneeFlexionRange());
    stroke = HydraulicAnalysis.r_ball * theta_range_rad;  % [m]
end

% =========================================================
%  STEP 12 — MINIMUM AND MAXIMUM CYLINDER PROCUREMENT LENGTHS
% =========================================================
% Assuming a SYMMETRIC posterior cylinder (both attachment pivots at
% radius r_ball from the knee joint axis), the centre-to-centre
% distance between the two clevis pins at any joint angle theta is
% given by the CHORD of a circle of radius r_ball:
%
%   L_cyl(theta) = 2 × r_ball × sin(theta / 2)
%
% This comes directly from the law of cosines with r1 = r2 = r_ball:
%   L² = r² + r² − 2·r²·cos(theta)  →  L = 2r·sin(theta/2)
%
% At minimum gait flexion (theta_min ≈ 50.3°):
%   L_cyl_min = 2 × 40 × sin(25.15°) ≈ 33.9 mm   ← cylinder retracted
%
% At maximum gait flexion (theta_max ≈ 112°):
%   L_cyl_max = 2 × 40 × sin(56°)   ≈ 66.3 mm   ← cylinder extended
%
% Required procurement stroke ≥ L_cyl_max − L_cyl_min ≈ 32.4 mm
%
% Assumption: pivot arms on thigh and shank are equal length (r_ball).
% If the arms differ in length, use the full law of cosines:
%   L = sqrt(r1² + r2² − 2·r1·r2·cos(theta))
function lengths = getCylinderLengths()
    extremes      = HydraulicAnalysis.getJointAngleExtremes();
    r             = HydraulicAnalysis.r_ball;
    theta_min_rad = deg2rad(extremes.theta_min_deg);
    theta_max_rad = deg2rad(extremes.theta_max_deg);
    % Chord = centre-to-centre distance between the two pivot pins
    lengths.L_cyl_min_mm    = 2 * r * sin(theta_min_rad / 2) * 1000;           % [mm] cylinder retracted
    lengths.L_cyl_max_mm    = 2 * r * sin(theta_max_rad / 2) * 1000;           % [mm] cylinder extended
    lengths.stroke_chord_mm = lengths.L_cyl_max_mm - lengths.L_cyl_min_mm;     % [mm] required stroke
end

% =========================================================
%  MAIN — RUN FULL HYDRAULIC UNIT ANALYSIS
% =========================================================
% Pass the effective needle valve diameter d_eff [m] from HydraulicNeedleValve.
% Returns a struct with all sizing results.
function results = getResults(d_eff)

    % Step 1: maximum axial load
    results.F_load_max_N      = HydraulicAnalysis.getMaxLoad();

    % Step 2: kinematics
    vel = HydraulicAnalysis.getMaxVelocity();
    results.omega_max_rads    = vel.omega_max;
    results.v_p_max_ms        = vel.v_p_max;

    % Step 3: cylinder bore
    results.D_min_calc_mm     = HydraulicAnalysis.getMinCylinderDiameter() * 1000;
    results.D_standard_mm     = HydraulicAnalysis.D_standard * 1000;

    % Step 4: pressure verification
    results.delta_p_req_MPa   = HydraulicAnalysis.getRequiredPressure() / 1e6;

    % Step 5: flow rate
    results.Q_max_mLs         = HydraulicAnalysis.getMaxFlowRate() * 1e6;

    % Step 6-7: fluid properties at 30 °C
    results.rho_30_kgm3       = HydraulicAnalysis.getFluidDensity30();
    results.mu_Pas             = HydraulicAnalysis.getDynamicViscosity();

    % Step 8: restriction channel length
    results.L_restriction_mm  = HydraulicAnalysis.getRestrictionLength(d_eff) * 1000;

    % Step 9: servo needle force
    results.F_needle_N        = HydraulicAnalysis.getNeedleForce(d_eff);

    % Step 10-11: knee flexion range and cylinder stroke from gait data; what cylinder travel length i need to get
    results.theta_range_deg   = HydraulicAnalysis.getKneeFlexionRange();
    results.stroke_mm         = HydraulicAnalysis.getStroke() * 1000;

    % Step 12: cylinder procurement lengths — chord formula, symmetric r_ball pivots
    cyl_lengths = HydraulicAnalysis.getCylinderLengths();
    results.L_cyl_min_mm = cyl_lengths.L_cyl_min_mm;   % retracted (most extended knee) [mm]
    results.L_cyl_max_mm = cyl_lengths.L_cyl_max_mm;   % extended  (most flexed knee)   [mm]

end

% =========================================================
%  GAIT PRESSURE TIME-SERIES — frame-by-frame hydraulics
% =========================================================
% At each gait frame, the knee joint angular velocity omega_joint(i)
% drives piston motion at:
%   v_p(i) = r_ball × |omega_joint(i)|
%
% The resulting piston flow rate is:
%   Q(i) = A_p × v_p(i)
%
% Pressure via Hagen-Poiseuille across the restriction channel:
%   P(i) = 128 × mu × L × Q(i) / (pi × d_eff^4)
%
% This demonstrates VARIABLE STIFFNESS — pressure (and hence the
% resistive torque) rises with knee angular velocity, automatically
% providing more resistance during fast motion (swing) and less
% during slow loading (stance).
%
% Derived quantities:
%   T_damp(i)    = P(i) × A_p × r_ball   — damping torque  [N·m]
%   F_needle(i)  = P(i) × A_seat         — servo needle force [N]
%
% Input: d_eff [m] — needle valve orifice diameter from HydraulicNeedleValve
function series = getPressureTimeSeries(d_eff)
    rootDir    = fileparts(fileparts(which('HydraulicAnalysis')));
    kneeTable  = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinKnee.csv'));
    legTable   = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinLeg.csv'));

    omega_thigh = kneeTable.(matlab.lang.makeValidName('Omega (rad/s)'));  % thigh segment [rad/s]
    omega_shank = legTable.(matlab.lang.makeValidName('Omega (rad/s)'));   % shank segment [rad/s]
    omega_joint = omega_shank - omega_thigh;                               % knee joint angular velocity [rad/s]

    N       = length(omega_joint);
    A_p     = pi * HydraulicAnalysis.D_standard^2 / 4;    % piston area [m^2]
    A_seat  = pi * d_eff^2 / 4;                           % needle seat area [m^2]
    mu      = HydraulicAnalysis.getDynamicViscosity();     % dynamic viscosity [Pa·s]
    L       = HydraulicAnalysis.getRestrictionLength(d_eff); % restriction length [m]

    Q_series = A_p * HydraulicAnalysis.r_ball * abs(omega_joint);       % flow rate [m^3/s]
    P_series = 128 * mu * L * Q_series ./ (pi * d_eff^4);               % pressure [Pa]

    series.gait_pct        = (0:N-1)' / (N-1) * 100;
    series.Q_series        = Q_series;                                   % [m^3/s]
    series.P_series        = P_series;                                   % [Pa]
    series.T_damp_series   = P_series * A_p * HydraulicAnalysis.r_ball; % [N·m]
    series.F_needle_series = P_series * A_seat;                         % [N]
end

end
end
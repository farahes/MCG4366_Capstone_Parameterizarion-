classdef ShaftAnalysis

properties (Constant)

    % STRESS CONCENTRATIONS
    q = 0.65;   % q factor for bending
    qs = 0.7;   % q factor for torsion
    Kt_b = 3.5; % stress concentration for step in bending
    Kt_t = 2;   % stress concentration for step in torsion
    Kf_k = 1.3; % fatigue stress concentration for keyway in bending and torsion

    % C FACTORS
    C_L = 1.0;  % load factor
    C_G = 0.9;  % gradient factor
    C_S = 0.6;  % surface factor
    C_T = 1.0;  % termperature factor
    C_R = 0.897;    % Reliability factor

    % FIXED SHAFT GEOMETRY
    lex_s = 0.006;  % [m] extra length at end of shaft for retaining ring and hangoff

end

methods (Static)

% --------SHAFT ANALYSIS--------

% FATIGUE LOADING OF SHAFT

% Resultant shear force on the shaft.
% The total JRF is shared equally by the two journal bearings (JRF/2 each component);
% the resultant is taken via Pythagoras across x and y.
function V = getV(JRFx, JRFy)
    V = sqrt((JRFy/2)^2 + (JRFx/2)^2);  % [N]
end

% Maximum bending moment at midspan between the two journal bearings.
% Modelled as a simply-supported beam:  M = V * L/2
function M = getM(L_s, V)
    M = (L_s/2)*V;  % [Nm]
end

% Torque on the shaft equals the peak knee joint moment,
% since the shaft directly transmits the joint's rotational resistance.
function T = getT(M_k)
    T = M_k;
end

% Corrected endurance limit (Juvinall-Marshek method).
% S'_n = 0.5*Su  (Moore rotating-beam baseline estimate)
% Sn = S'_n * C_L * C_G * C_S * C_T * C_R  (corrected for load, gradient, surface, temp, reliability)
function Sn = getSn(Su)
    Sn_prime = 0.5*Su;   % [MPa] S'_n (Moore rotating-beam endurance limit)
    Sn = Sn_prime*ShaftAnalysis.C_L*ShaftAnalysis.C_G*ShaftAnalysis.C_S*ShaftAnalysis.C_T*ShaftAnalysis.C_R;    % [MPa] corrected endurance limit
end

% Fatigue stress concentration factor via Neuber's method:  Kf = 1 + (Kt-1)*q
% q = notch sensitivity (0–1, material-dependent); Kt = theoretical geometric SCF.
function Kf = getKf(q, Kt)
    Kf = 1 + (Kt - 1)*q;
end

% ASME-Elliptic fatigue criterion — alternating load amplitude term.
% Combines alternating bending Ma and alternating torque Ta,
% weighted by their respective fatigue stress concentration factors.
function A = getA(Kf, Ma, Kfs, Ta)
    A = sqrt(4*(Kf*Ma)^2 + 3*(Kfs*Ta)^2);
end

% ASME-Elliptic fatigue criterion — mean load term.
% Combines mean bending Mm and mean torque Tm components.
function B = getB(Kf, Mm, Kfs, Tm)
    B = Kf*Mm + sqrt((Kf*Mm)^2 + (Kfs*Tm)^2);
end

% Minimum shaft diameter from the ASME-Elliptic fatigue failure criterion.
% Solves:  d^3 = (16*n/pi) * (A/Sn + B/Su)
% where Sn is the corrected endurance limit and Su is the ultimate tensile strength.
function d = getShaftDiameterFatigue(n, A, B, Sn, Su)
    d = (16*n*(A/Sn + B/Su)/pi())^(1/3);
end

% STATIC BENDING OF SHAFT
% Minimum diameter from static bending:  d^3 = 32*M / (pi * sigma_allowed)
% sigma_allowed = Sy / n  (yield stress with safety factor).
function d = getShaftDiameterBending(n, Sy, M)
    sigma = Sy/n;
    d = (32*M/(pi()*sigma))^(1/3);
end

% STATIC SHEAR OF SHAFT
% Minimum diameter from transverse shear:  d^3 = 16*V / (3*pi * tau_allowed)
% Based on maximum shear stress at the neutral axis of a solid circular section (= 4V/3A).
function d = getShaftDiameterShear(n, Ssy, V)
    tau = Ssy/n;
    d = (16*V/(3*pi()*tau))^(1/3);
end

% TORSIONAL DEFLECTION CHECK
% Verifies the total angle of twist across the two shaft segments stays ≤ 1°.
% theta = (32*T / (pi*G)) * Σ(L_i / D_i^4)   [polar moment J = pi*D^4/32 for solid shaft]
% Main.getResults increments D_s and d_s by 1 mm until this check passes.
function bool = getCheckTorsion(log, T, G, L1, D1, L2, D2)
    theta_allowed = deg2rad(1);   % 1 degree maximum allowable twist angle
    theta = (32*T/(pi()*G))*((L1/D1^4) + (L2/D2^4));
    bool = theta < theta_allowed;
    fprintf(log, 'Torsional deflection for D_s=%.2fmm L_s=%.2fmm d_s=%.2fmm l_s=%.2fmm: theta = %.2f degrees\n\n', D1*1000, L1*1000, D2*1000, L2*1000, rad2deg(theta));
end

% REQUIRED SHAFT DIAMETER

% Returns the governing (worst-case) shaft diameters across all failure modes.
% D_s  (large-section at keyway):   checked for ASME-Elliptic fatigue and static bending.
% d_s  (small-section at bearings): checked for ASME-Elliptic fatigue and static shear.
% The larger value from each pair of checks is returned as the required diameter.
function shaftDiameter = getShaftDiameter(log, n, Su, Sy, Ssy, JRFx, JRFy, M_k, L_s)

    fprintf(log, 'Limiting Shaft Diameters:\n');

    % Determine limiting D_s (large-diameter section at keyway/bending region)
    % Fatigue
    D1 = ShaftAnalysis.getShaftDiameterFatigue( ...
        n, ...
        ShaftAnalysis.getA(ShaftAnalysis.Kf_k, ShaftAnalysis.getM(L_s, ShaftAnalysis.getV(JRFx, JRFy)), ShaftAnalysis.Kf_k, ShaftAnalysis.getT(M_k)), ...
        ShaftAnalysis.getB(ShaftAnalysis.Kf_k, 0, ShaftAnalysis.Kf_k, 0), ...
        ShaftAnalysis.getSn(Su), ...
        Su ...
        );
    fprintf(log, 'D_s (fatigue loading) = %.2f mm\n', D1*1000);
    % Pure bending
    D2 = ShaftAnalysis.getShaftDiameterBending(n, Sy, ShaftAnalysis.getM(L_s, ShaftAnalysis.getV(JRFx, JRFy)));
    fprintf(log, 'D_s (bending stress) = %.2f mm\n', D2*1000);

    % Determine limiting d_s
    % Fatigue
    d1 = ShaftAnalysis.getShaftDiameterFatigue( ...
        n, ...
        ShaftAnalysis.getA(ShaftAnalysis.getKf(ShaftAnalysis.q, ShaftAnalysis.Kt_b), 0, ShaftAnalysis.getKf(ShaftAnalysis.qs, ShaftAnalysis.Kt_t), ShaftAnalysis.getT(M_k)), ...
        ShaftAnalysis.getB(ShaftAnalysis.getKf(ShaftAnalysis.q, ShaftAnalysis.Kt_b), 0, ShaftAnalysis.getKf(ShaftAnalysis.qs, ShaftAnalysis.Kt_t), 0), ...
        ShaftAnalysis.getSn(Su), ...
        Su ...
        );
    fprintf(log, 'd_s (fatigue loading) = %.2f mm\n', d1*1000);
    % Static shear
    d2 = ShaftAnalysis.getShaftDiameterShear( ...
        n, ...
        Ssy, ...
        ShaftAnalysis.getV(JRFx, JRFy) ...
        );
    fprintf(log, 'd_s (shear loading) = %.2f mm\n', d2*1000);

    shaftDiameter.D = max([D1, D2]);
    shaftDiameter.d = max([d1, d2]);
    fprintf(log, 'Limiting d_s = %.2f mm\n', shaftDiameter.d*1000);
    fprintf(log, 'Limiting D_s = %.2f mm\n', shaftDiameter.D*1000);
    
end

% --------JOURNAL BEARING--------

function JB = getJB(JRF, d_s)
    jbDimensions = JournalBearing.getJournalBearingDimensions(JRF, d_s);    % search journal bearing catalogue

    JB.id = jbDimensions.shaftD;    % diameter of the shaft
    JB.od = jbDimensions.housingID; % inner diameter of the housing
    JB.l = jbDimensions.lngth;  % length of the journal bearing
    JB.part = jbDimensions.prt; % part number from McMasterCarr
end

% Adjust shaft dimensions to match the selected journal bearing.
% l_s = L_s (large section) + 2*lex_s (retaining ring clearance, one each end)
%           + 2 * JB.length  (one bearing seated at each end of the small-diameter section).
% d_s is set equal to the journal bearing bore ID.
function shaftDimensions = getFinalShaftDimensions(JBdim, L_s)
    shaftDimensions.l_s = L_s + 2*ShaftAnalysis.lex_s + 2*JBdim.l;
    shaftDimensions.d_s = JBdim.id;
end

% --------KEY DIMENSION--------
% Same material as the shaft (1045 HR Carbon Steel).
% Square key width = D/4, per standard key proportions (Shigley's Table 7-6).
% The key transmits torque between the shaft and the prosthetic socket coupling.
function w_k = getWk(D)
    w_k = 0.25*D;
end

end
end
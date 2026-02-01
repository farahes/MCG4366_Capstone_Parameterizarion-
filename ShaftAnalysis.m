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

    % SHAFT GEOMETRY
    L_s = 0.08; % [m] length of the large diameter of the shaft
    lex_s = 0.006;  % [m] extra length at end of shaft for retaining ring and hangoff

end

methods (Static)

% --------SHAFT ANALYSIS--------

% FATIGUE LOADING OF SHAFT

% magnitude of the maximmum shear force on the shaft
function V = getV(JRFx, JRFy)
    V = sqrt((JRFy/2)^2 + (JRFx/2)^2);  % [N]
end

% magnitude of the maximum bending moment on the shaft
function M = getM(L, V)
    M = (L/2)*V;  % [Nm]
end

% magnitude of the torque on the shaft
function T = getT(M_k)
    T = M_k;
end

function Sn = getSn(Su)
    Sn_prime = 0.5*Su;   % [MPa] S'_n (Moore endurance limit)
    Sn = Sn_prime*ShaftAnalysis.C_L*ShaftAnalysis.C_G*ShaftAnalysis.C_S*ShaftAnalysis.C_T*ShaftAnalysis.C_R;    % [MPa] endurance limit (10^3-cycle strength)
end

% fatigue stress concentration for step
function Kf = getKf(q, Kt)
    Kf = 1 + (Kt - 1)*q;
end

function A = getA(Kf, Ma, Kfs, Ta)
    A = sqrt(4*(Kf*Ma)^2 + 3*(Kfs*Ta)^2);
end

function B = getB(Kf, Mm, Kfs, Tm)
    B = Kf*Mm + sqrt((Kf*Mm)^2 + (Kfs*Tm)^2);
end

function d = getShaftDiameterFatigue(n, A, B, Sn, Su)
    d = (16*n*(A/Sn + B/Su)/pi())^(1/3);
end

% STATIC BENDING OF SHAFT

function d = getShaftDiameterBending(n, Sy, M)
    sigma = Sy/n;
    d = (32*M/(pi()*sigma))^(1/3);
end

% STATIC SHEAR OF SHAFT

function d = getShaftDiameterShear(n, Ssy, V)
    tau = Ssy/n;
    d = (16*V/(3*pi()*tau))^(1/3);
end

% TORSIONAL DEFLECTION OF SHAFT
function bool = getCheckTorsion(T, G, L1, D1, L2, D2)
    theta_allowed = deg2rad(1);
    theta = (32*T/(pi()*G))*((L1/D1^4) + (L2/D2^4));
    bool = theta < theta_allowed;
end

% REQUIRED SHAFT DIAMETER

% maximum of the required shaft diameters for each loading condition
function shaftDiameter = getShaftDiameter(n, Su, Sy, Ssy, JRFx, JRFy, M_k)

    % Determine limiting D_s
    % Fatigue
    D1 = ShaftAnalysis.getShaftDiameterFatigue( ...
        n, ...
        ShaftAnalysis.getA(ShaftAnalysis.Kf_k, ShaftAnalysis.getM(ShaftAnalysis.L_s, ShaftAnalysis.getV(JRFx, JRFy)), ShaftAnalysis.Kf_k, ShaftAnalysis.getT(M_k)), ...
        ShaftAnalysis.getB(ShaftAnalysis.Kf_k, 0, ShaftAnalysis.Kf_k, 0), ...
        ShaftAnalysis.getSn(Su), ...
        Su ...
        );
    % Pure bending
    D2 = ShaftAnalysis.getShaftDiameterBending(n, Sy, ShaftAnalysis.getM(ShaftAnalysis.L_s, ShaftAnalysis.getV(JRFx, JRFy)));

    % Determine limiting d_s
    % Fatigue
    d1 = ShaftAnalysis.getShaftDiameterFatigue( ...
        n, ...
        ShaftAnalysis.getA(ShaftAnalysis.getKf(ShaftAnalysis.q, ShaftAnalysis.Kt_b), 0, ShaftAnalysis.getKf(ShaftAnalysis.qs, ShaftAnalysis.Kt_t), ShaftAnalysis.getT(M_k)), ...
        ShaftAnalysis.getB(ShaftAnalysis.getKf(ShaftAnalysis.q, ShaftAnalysis.Kt_b), 0, ShaftAnalysis.getKf(ShaftAnalysis.qs, ShaftAnalysis.Kt_t), 0), ...
        ShaftAnalysis.getSn(Su), ...
        Su ...
        );
    % Static shear
    d2 = ShaftAnalysis.getShaftDiameterShear( ...
        n, ...
        Ssy, ...
        ShaftAnalysis.getV(JRFx, JRFy) ...
        );

    shaftDiameter.D = max([D1, D2]);
    shaftDiameter.d = max([d1, d2]);

    % check static deflection and get critical speed?
    % critical speed > 2xoperating speed
end

% --------JOURNAL BEARING--------

function JB = getJB(JRF, d_s)
    jbDimensions = JournalBearing.getJournalBearingDimensions(JRF, d_s);    % search journal bearing catalogue

    JB.id = jbDimensions.shaftD;    % diameter of the shaft
    JB.od = jbDimensions.housingID; % inner diameter of the housing
    JB.l = jbDimensions.lngth;  % length of the journal bearing
    JB.part = jbDimensions.prt; % part number from McMasterCarr
end

function shaftDimensions = getFinalShaftDimensions(JBdim)
    shaftDimensions.l_s = ShaftAnalysis.L_s + 2*ShaftAnalysis.lex_s + 2*JBdim.l;
    shaftDimensions.d_s = JBdim.id;
end

% --------KEY DIMENSION--------
% same material as shaft

function w_k = getWk(D)
    w_k = 0.25*D;
end

end
end
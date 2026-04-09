%{

Subscript Convention
f : foot
l : leg
a : ankle
k : knee
g : ground

%}

classdef JointReactionForce

properties (Constant)

    g = 9.81; % [m/s^2]
    RoG_l = 0.302; % radius of gyration of the leg about its COM
    RoG_f = 0.475;   % radius of gyration of the foot about its COM

end

methods (Static)

% -------- ANTHROPOMETRIC DATA --------

% import Winter's Appendix data
function footData = getFootData()
    persistent footTable
    if isempty(footTable)
        rootDir = fileparts(fileparts(which('JointReactionForce')));
        footTable = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinFoot.csv'));
    end
    footData = footTable;
end

function legData = getLegData()
    persistent legTable
    if isempty(legTable)
        rootDir = fileparts(fileparts(which('JointReactionForce')));
        legTable = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinLeg.csv'));
    end
    legData = legTable;
end

function ankleData = getAnkleData()
    persistent ankleTable
    if isempty(ankleTable)
        rootDir = fileparts(fileparts(which('JointReactionForce')));
        ankleTable = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinAnkle.csv'));
    end
    ankleData = ankleTable;
end

function kneeData = getKneeData()
    persistent kneeTable
    if isempty(kneeTable)
        rootDir = fileparts(fileparts(which('JointReactionForce')));
        kneeTable = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinKnee.csv'));
    end
    kneeData = kneeTable;
end

function COPData = getCOPData()
    persistent COPTable
    if isempty(COPTable)
        rootDir = fileparts(fileparts(which('JointReactionForce')));
        COPTable = readtable(fullfile(rootDir, 'data', 'WinterAppendix_COP.csv'));
    end
    COPData = COPTable;
end

function GRFData = getGRFData()
    persistent GRFTable
    if isempty(GRFTable)
        rootDir = fileparts(fileparts(which('JointReactionForce')));
        GRFTable = readtable(fullfile(rootDir, 'data', 'WinterAppendix_GRF.csv'));
    end
    GRFData = GRFTable;
end

% Foot dimensions
function m_f = getMf(BW)
    m_f = 0.0145*BW; % [kg] mass of the foot
end

function l_f = getLf(H)
    l_f = 0.152*H;   % [m] length of the foot
end

% Lower leg dimensions
function m_l = getMl(BW)
    m_l = 0.0465*BW;   % [kg] mass of the lower leg
end

function l_l = getLl(H)
    l_l = 0.246*H; % [m] length of the lower leg (ankle to knee)
end

% Moment of inertia about the CoG
function I = getI(m, l, RoG)
    I = m*(l*RoG)^2;   % [kg*m^2]
end

% -------- JRF - INVERSE DYNAMICS --------

% Ankle joint reaction force via Newton's 2nd law applied to the foot segment.
% ΣF = m*a  →  F_ankle = m_f * a_foot - F_GRF  (gravity term added to y-component)
% F_GRF acts upward from the ground; F_ankle is the force transmitted from the leg.
function F_a = getFAnkle(m_f, F_gx, F_gy, a_fx, a_fy)
    F_a.x = m_f*a_fx - F_gx;
    F_a.y = m_f*a_fy - F_gy + m_f*JointReactionForce.g;
end

% Ankle joint moment via Euler's rotational equation about the foot CoM.
% ΣM_CoM = I*α  →  all cross-products are 2D (z-component only): r×F = rx*Fy - ry*Fx
% Moment arms are computed from CoM to each joint/COP position.
function M_a = getMAnkle(I_f, alpha_f, foot_x, foot_y, ankle_x, ankle_y, COP_x, COP_y, F_ax, F_ay, F_gx, F_gy)
    M_a = I_f*alpha_f + (foot_x-ankle_x)*F_ay - (foot_y-ankle_y)*F_ax + (foot_x-COP_x)*F_gy - (foot_y-COP_y)*F_gx;
end

% Knee joint reaction force via Newton's 2nd law on the lower leg segment.
% By Newton's 3rd law, the lower leg experiences +F_ankle (reaction to foot-on-leg force).
% F_knee = m_l * a_leg + F_ankle  (+ gravity component in y-direction)
function F_k = getFKnee(m_l, F_ax, F_ay, a_lx, a_ly)
    F_k.x = m_l*a_lx + F_ax;
    F_k.y = m_l*a_ly + F_ay + m_l*JointReactionForce.g;
end

% Knee joint moment via Euler's rotational equation on the lower leg CoM.
% M_knee propagates M_ankle up the chain and adds the cross-products of:
%   r_knee (from CoM to knee) × F_knee  and  r_ankle (from CoM to ankle) × (-F_ankle)
% This is the internal moment the prosthetic knee joint must resist during gait.
function M_k = getMKnee(I_l, alpha_l, leg_x, leg_y, knee_x, knee_y, ankle_x, ankle_y, F_ax, F_ay, F_kx, F_ky, M_a)
    M_k = I_l*alpha_l + M_a + (leg_x-knee_x)*F_ky - (leg_y-knee_y)*F_kx - (leg_x-ankle_x)*F_ay + (leg_y-ankle_y)*F_ax;
end

% Get the maximum JRF during gait
function maxJRF = getMaxJRF(BW, H)

    % create log file
    rootDir = fileparts(fileparts(which('JointReactionForce')));
    log = fopen(fullfile(rootDir, 'logs', 'JRFLogFile.txt'), 'w');
    if log == -1
        error('Could not create log file');
    end
    fprintf(log, '/***************************************/\n');
    fprintf(log, '/                JRF LOG                /\n');
    fprintf(log, '/***************************************/\n\n');

    % print patient data
    fprintf(log, 'Patient body weight: %.2f kg\n', BW);
    fprintf(log, 'Patient height: %.2f m\n\n', H);

    fprintf(log, 'Frame\t\tFa_x\t\tFa_y\t\tMa\t\tFk_x\t\tFk_y\t\tMk\t\tF_k total\n');

    GRFTable = JointReactionForce.getGRFData();
    GRFx = GRFTable.(matlab.lang.makeValidName('GRF_x, normalized (N/kg)'));
    GRFy = GRFTable.(matlab.lang.makeValidName('GRF_y, normalized (N/kg)'));

    FootTable = JointReactionForce.getFootData();
    a_fx = FootTable.(matlab.lang.makeValidName('Acc_x (m/s^2)'));
    a_fy = FootTable.(matlab.lang.makeValidName('Acc_y (m/s^2)'));
    alpha_f = FootTable.(matlab.lang.makeValidName('Alpha (rad/s^2)'));
    foot_x = FootTable.(matlab.lang.makeValidName('CofM_x (m)'));
    foot_y = FootTable.(matlab.lang.makeValidName('CofM_y (m)'));

    AnkleTable = JointReactionForce.getAnkleData();
    ankle_x = AnkleTable.(matlab.lang.makeValidName('Ankle_x (m)'));
    ankle_y = AnkleTable.(matlab.lang.makeValidName('Ankle_y (m)'));

    COPTable = JointReactionForce.getCOPData();
    COP_x = COPTable.(matlab.lang.makeValidName('COP_x (m)'));

    LegTable = JointReactionForce.getLegData();
    a_lx = LegTable.(matlab.lang.makeValidName('Acc_x (m/s^2)'));
    a_ly = LegTable.(matlab.lang.makeValidName('Acc_y (m/s^2)'));
    alpha_l = LegTable.(matlab.lang.makeValidName('Alpha (rad/s^2)'));
    leg_x = LegTable.(matlab.lang.makeValidName('CofM_x (m)'));
    leg_y = LegTable.(matlab.lang.makeValidName('CofM_y (m)'));

    KneeTable = JointReactionForce.getKneeData();
    knee_x = KneeTable.(matlab.lang.makeValidName('Knee_x (m)'));
    knee_y = KneeTable.(matlab.lang.makeValidName('Knee_y (m)'));

    % set initial maximums
    F_k_max_total = 0;
    F_k_max.x = 0;
    F_k_max.y = 0;
    M_k_max = 0;

    % parse gait cycle for maximums
    for i = 1:length(GRFx)
        F_a = JointReactionForce.getFAnkle( ...
            JointReactionForce.getMf(BW), ...
            GRFx(i)*BW, ...
            GRFy(i)*BW, ...
            a_fx(i), ...
            a_fy(i) ...
            );
        M_a = JointReactionForce.getMAnkle( ...
            JointReactionForce.getI(JointReactionForce.getMf(BW), JointReactionForce.getLf(H), JointReactionForce.RoG_f), ...
            alpha_f(i), ...
            foot_x(i), ...
            foot_y(i), ...
            ankle_x(i), ...
            ankle_y(i), ...
            COP_x(i), ...
            0, ...
            F_a.x, ...
            F_a.y, ...
            GRFx(i)*BW, ...
            GRFy(i)*BW ...
            );
        F_k = JointReactionForce.getFKnee( ...
            JointReactionForce.getMl(BW), ...
            F_a.x, ...
            F_a.y, ...
            a_lx(i), ...
            a_ly(i) ...
            );
        M_k = JointReactionForce.getMKnee( ...
            JointReactionForce.getI(JointReactionForce.getMl(BW), JointReactionForce.getLl(H), JointReactionForce.RoG_l), ...
            alpha_l(i), ...
            leg_x(i), ...
            leg_y(i), ...
            knee_x(i), ...
            knee_y(i), ...
            ankle_x(i), ...
            ankle_y(i), ...
            F_a.x, ...
            F_a.y, ...
            F_k.x, ...
            F_k.y, ...
            M_a ...
            );
        F_k_total = sqrt(F_k.x^2 + F_k.y^2);

        fprintf(log, '%d\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\t\t%.2f\n', i, F_a.x, F_a.y, M_a, F_k.x, F_k.y, M_k, F_k_total);

        % check max
        if F_k_total > F_k_max_total
            F_k_max_total = F_k_total;
            F_k_max.x = F_k.x;
            F_k_max.y = F_k.y;
        end
%%The knee moment during gait swings negative (flexion) and positive (extension). 
% If the largest-magnitude moment happens to be negative, M_k_max stays at 0.
        if abs(M_k) > abs(M_k_max) 
            M_k_max = M_k;
        end
    end

    maxJRF.Fx = F_k_max.x;
    maxJRF.Fy = F_k_max.y;
    maxJRF.M = M_k_max;
end

% =========================================================
%  FULL GAIT TIME-SERIES — all frame-by-frame values
% =========================================================
% Runs the same inverse-dynamics loop as getMaxJRF but instead of
% tracking only the peak values it stores every frame's output.
% Also reads the thigh and shank segment angles/velocities to compute
% the true KNEE JOINT angle at each frame:
%   theta_joint = theta_shank - theta_thigh  (Winter's sign convention)
%   omega_joint = omega_shank - omega_thigh
%
% This time-series is used by GaitPlots to generate all 6 gait cycle plots.
%
% Returns struct with arrays of length N (number of gait frames):
%   .gait_pct    — gait cycle percentage 0-100%
%   .Fx_k        — horizontal knee JRF [N]
%   .Fy_k        — vertical knee JRF [N]
%   .F_k_total   — resultant knee JRF [N]
%   .M_k         — knee internal moment [N·m]
%   .theta_joint — knee joint angle [deg]
%   .omega_joint — knee joint angular velocity [rad/s]
function series = getGaitTimeSeries(BW, H)

    % Load all data tables (persistent cache: files are read only once per session)
    GRFTable = JointReactionForce.getGRFData();
    GRFx = GRFTable.(matlab.lang.makeValidName('GRF_x, normalized (N/kg)'));
    GRFy = GRFTable.(matlab.lang.makeValidName('GRF_y, normalized (N/kg)'));

    FootTable = JointReactionForce.getFootData();
    a_fx    = FootTable.(matlab.lang.makeValidName('Acc_x (m/s^2)'));
    a_fy    = FootTable.(matlab.lang.makeValidName('Acc_y (m/s^2)'));
    alpha_f = FootTable.(matlab.lang.makeValidName('Alpha (rad/s^2)'));
    foot_x  = FootTable.(matlab.lang.makeValidName('CofM_x (m)'));
    foot_y  = FootTable.(matlab.lang.makeValidName('CofM_y (m)'));

    AnkleTable = JointReactionForce.getAnkleData();
    ankle_x = AnkleTable.(matlab.lang.makeValidName('Ankle_x (m)'));
    ankle_y = AnkleTable.(matlab.lang.makeValidName('Ankle_y (m)'));

    COPTable = JointReactionForce.getCOPData();
    COP_x = COPTable.(matlab.lang.makeValidName('COP_x (m)'));

    LegTable = JointReactionForce.getLegData();
    a_lx        = LegTable.(matlab.lang.makeValidName('Acc_x (m/s^2)'));
    a_ly        = LegTable.(matlab.lang.makeValidName('Acc_y (m/s^2)'));
    alpha_l     = LegTable.(matlab.lang.makeValidName('Alpha (rad/s^2)'));
    leg_x       = LegTable.(matlab.lang.makeValidName('CofM_x (m)'));
    leg_y       = LegTable.(matlab.lang.makeValidName('CofM_y (m)'));
    shank_theta = LegTable.(matlab.lang.makeValidName('Theta (deg)'));
    omega_shank = LegTable.(matlab.lang.makeValidName('Omega (rad/s)'));

    KneeTable = JointReactionForce.getKneeData();
    knee_x     = KneeTable.(matlab.lang.makeValidName('Knee_x (m)'));
    knee_y     = KneeTable.(matlab.lang.makeValidName('Knee_y (m)'));

    % Thigh segment angle and omega — needed for true knee joint angle
    % (KneeTable stores THIGH segment data; joint angle = shank - thigh)
    rootDir     = fileparts(fileparts(which('JointReactionForce')));
    kneeFullTab = readtable(fullfile(rootDir, 'data', 'WinterAppendix_KinKnee.csv'));
    thigh_theta = kneeFullTab.(matlab.lang.makeValidName('Theta (deg)'));
    omega_thigh = kneeFullTab.(matlab.lang.makeValidName('Omega (rad/s)'));

    % Pre-compute invariant quantities
    m_f = JointReactionForce.getMf(BW);
    m_l = JointReactionForce.getMl(BW);
    I_f = JointReactionForce.getI(m_f, JointReactionForce.getLf(H), JointReactionForce.RoG_f);
    I_l = JointReactionForce.getI(m_l, JointReactionForce.getLl(H), JointReactionForce.RoG_l);

    N = length(GRFx);
    series.gait_pct    = (0:N-1)' / (N-1) * 100;
    series.Fx_k        = zeros(N, 1);
    series.Fy_k        = zeros(N, 1);
    series.F_k_total   = zeros(N, 1);
    series.M_k         = zeros(N, 1);
    series.theta_joint = shank_theta - thigh_theta;   % true knee joint angle [deg]
    series.omega_joint = omega_shank  - omega_thigh;  % true knee angular velocity [rad/s]

    for i = 1:N
        F_a = JointReactionForce.getFAnkle(m_f, GRFx(i)*BW, GRFy(i)*BW, a_fx(i), a_fy(i));
        M_a = JointReactionForce.getMAnkle(I_f, alpha_f(i), foot_x(i), foot_y(i), ...
              ankle_x(i), ankle_y(i), COP_x(i), 0, F_a.x, F_a.y, GRFx(i)*BW, GRFy(i)*BW);
        F_k = JointReactionForce.getFKnee(m_l, F_a.x, F_a.y, a_lx(i), a_ly(i));
        M_k = JointReactionForce.getMKnee(I_l, alpha_l(i), leg_x(i), leg_y(i), ...
              knee_x(i), knee_y(i), ankle_x(i), ankle_y(i), ...
              F_a.x, F_a.y, F_k.x, F_k.y, M_a);

        series.Fx_k(i)      = F_k.x;
        series.Fy_k(i)      = F_k.y;
        series.F_k_total(i) = sqrt(F_k.x^2 + F_k.y^2);
        series.M_k(i)       = M_k;
    end
end

end 
end
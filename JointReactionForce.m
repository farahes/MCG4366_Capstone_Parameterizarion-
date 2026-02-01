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
        footTable = readtable('WinterAppendix_KinFoot.csv');
    end
    footData = footTable;
end

function legData = getLegData()
    persistent legTable
    if isempty(legTable)
        legTable = readtable('WinterAppendix_KinLeg.csv');
    end
    legData = legTable;
end

function ankleData = getAnkleData()
    persistent ankleTable
    if isempty(ankleTable)
        ankleTable = readtable('WinterAppendix_KinAnkle.csv');
    end
    ankleData = ankleTable;
end

function kneeData = getKneeData()
    persistent kneeTable
    if isempty(kneeTable)
        kneeTable = readtable('WinterAppendix_KinKnee.csv');
    end
    kneeData = kneeTable;
end

function COPData = getCOPData()
    persistent COPTable
    if isempty(COPTable)
        COPTable = readtable('WinterAppendix_COP.csv');
    end
    COPData = COPTable;
end

function GRFData = getGRFData()
    persistent GRFTable
    if isempty(GRFTable)
        GRFTable = readtable('WinterAppendix_GRF.csv');
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

function r = getR(x1, y1, x2, y2)
    rx = abs(x1-x2);
    ry = abs(y1-y2);
    r = sqrt(rx^2 + ry^2);
end

% Forces at the ankle
function F_a = getFAnkle(m_f, F_gx, F_gy, a_fx, a_fy)
    F_a.x = m_f*a_fx - F_gx;
    F_a.y = m_f*a_fy - F_gy + m_f*JointReactionForce.g;
end

function M_a = getMAnkle(I_f, alpha_f, foot_x, foot_y, ankle_x, ankle_y, COP_x, COP_y, F_ax, F_ay, F_gx, F_gy)
    M_a = I_f*alpha_f + (foot_x-ankle_x)*F_ay - (foot_y-ankle_y)*F_ax + (foot_x-COP_x)*F_gy - (foot_y-COP_y)*F_gx;
end

% Forces on the knee
function F_k = getFKnee(m_l, F_ax, F_ay, a_lx, a_ly)
    F_k.x = m_l*a_lx + F_ax;
    F_k.y = m_l*a_ly + F_ay + m_l*JointReactionForce.g;
end

function M_k = getMKnee(I_l, alpha_l, leg_x, leg_y, knee_x, knee_y, ankle_x, ankle_y, F_ax, F_ay, F_kx, F_ky, M_a)
    M_k = I_l*alpha_l + M_a + (leg_x-knee_x)*F_ky - (leg_y-knee_y)*F_kx - (leg_x-ankle_x)*F_ay + (leg_y-ankle_y)*F_ax;
end

% Force along (y') and perpendicular (x') to the axis of the prosthesis
% **MAY NOT NEED THIS*** PLEASE DON"T NEED THIS 
% ***DOUBLE CHECK ANGLE SIGN CONVENTIONS***
function JRF_prime = getJRFprime(JRF_x, JRF_y, theta_l)
    theta_prime = deg2rad(theta_l) - pi()/2;
    JRF_prime.x_prime = JRF_y*sin(theta_prime) + JRF_x*cos(-theta_prime);
    JRF_prime.y_prime = JRF_y*cos(theta_prime) + JRF_x*sin(-theta_prime);
end

% Get the maximum JRF during gait
function maxJRF = getMaxJRF(BW, H)
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
            GRFx(i), ...
            GRFy(i) ...
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

        % check max
        if F_k_total > F_k_max_total
            F_k_max_total = F_k_total;
            F_k_max.x = F_k.x;
            F_k_max.y = F_k.y;
        end

        if M_k > M_k_max
            M_k_max = M_k;
        end
    end

    maxJRF.Fx = F_k_max.x;
    maxJRF.Fy = F_k_max.y;
    maxJRF.M = M_k_max;
end

end 
end
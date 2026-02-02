%{

Subscript Convention
s : shaft
su : supports
fr: frame
lp : lip
b : ball

%}


classdef FrameAnalysis

properties (Constant)
    r_ball = 0.04; % [m], radius of the ball
    w_ball = 0.08;  % [m], width of the ball
    phi1 = 2*pi()/3;   % [rad], angle between the front legs and back legs
    phi2 = pi()/3; % [rad], angle between front leg and the lip
    L_fr = 0.12; % [m], length of the legs
    ratio = 0.3;    % ratio between a and b
end

methods (Static)

function tau_allowed = getTau(Ssy, n)
    tau_allowed = Ssy/n;
end

function sigma_allowed = getSigma(Sy, n)
    sigma_allowed = Sy/n;
end

% -------- SUPPORTS --------

% BUCKLING
function Pcr = getPcrsu(JRF, n)
    Pcr = (JRF/2)*n;
end

function H_su = getHsu(d_isu)
    H_su = ((FrameAnalysis.r_ball-d_isu)/2)*cos(FrameAnalysis.phi1/2);
end

function t_su = gettsuBuckling(n, JRF, E, d_isu)
    Pcr = FrameAnalysis.getPcrsu(JRF, n);
    H_su = FrameAnalysis.getHsu(d_isu);
    t_su = (3*Pcr*H_su/(pi()^2*E*d_isu))^(1/3);
end

% SHEAR (x-y plane)
function t_su = gettsuShearXY(JRF, d_isu, tau)
    t_su = (3/4)*JRF/(d_isu*tau);
end 

% BENDING
function w_su = wsufunction(d_isu, h_su)
    w_su = d_isu + 2*h_su*tan(FrameAnalysis.phi1/2);
end

function M = MfunctionBending(JRF, h_su)
    M = JRF*(1-h_su)/2;
end

function t_su = gettsuBendingXY(JRF, H_su, d_isu, sigma)
    M_max = JRF*H_su/2;
    t_su = 6*M_max/(sigma*d_isu^2);
end

function t_su = gettsuBendingYZ(JRF, H_su, d_isu, sigma)
    M_max = JRF*H_su;
    t_su = sqrt(6*M_max/(sigma*d_isu));
end

% REQUIRED SUPPORT THICKNESS

function t_su = getSupportThickness(n, JRF, E, Ssy, Sy, d_isu)

    % Buckling
    t1 = FrameAnalysis.gettsuBuckling(n, JRF, E, d_isu);
    % Shear
    tau = FrameAnalysis.getTau(Ssy, n);
    t2 = FrameAnalysis.gettsuShearXY(JRF, d_isu, tau);
    % Bending
    sigma = FrameAnalysis.getSigma(Sy, n);
    t3 = FrameAnalysis.gettsuBendingXY(JRF, FrameAnalysis.getHsu(d_isu), d_isu, sigma);
    t4 = FrameAnalysis.gettsuBendingYZ(JRF, FrameAnalysis.getHsu(d_isu), d_isu, sigma);

    t_su = max([t1, t2, t3, t4]);
end

% -------- LEGS --------

% BUCKLING
function Pcr = getPcrfr(JRF, n)
    Pcr = n*(JRF/4);
end

function I_allowed = getIAllowed(Pcr, E)
    I_allowed = Pcr*FrameAnalysis.L_fr^2/(4*E*pi()^2);
end

function legDim = getLegDimsBuckling(I_allowed)
    syms a real positive
    b = FrameAnalysis.ratio*a;
    cy = (a^2 - b^2 + a*b)/(4*a-2*b);
    I1 = (a*b^3)/12;
    A1 = a*b;
    d1 = cy - b/2;
    I2 = (b*(a-b)^3)/12;
    A2 = b*(a-b);
    d2 = b + (a-b)/2 - cy;

    I = I1 + I2 + A1*d1^2 + A2*d2^2;
    eqn = I == I_allowed;
    a_sol = vpasolve(eqn, a, I_allowed^(1/4));
    a_sol = double(a_sol);
    a_sol = a_sol(a_sol > 0 & imag(a_sol) == 0);
    a_sol = min(a_sol);
    b_sol = FrameAnalysis.ratio*a_sol;

    legDim.a = a_sol;
    legDim.b = b_sol;
end
    
% SHEAR
function legDim = getLegDimsShear(JRF, tau_allowed)
    syms a real positive
    b = FrameAnalysis.ratio*a;
    tau = 3*JRF/(8*(a^2-(a-b)^2));
    eqn = tau == tau_allowed;
    a_sol = vpasolve(eqn, a, tau_allowed^(1/4));
    a_sol = double(a_sol);
    a_sol = a_sol(a_sol > 0 & imag(a_sol) == 0);
    a_sol = min(a_sol);
    b_sol = FrameAnalysis.ratio*a_sol;

    legDim.a = a_sol;
    legDim.b = b_sol;
end

% BENDING
function M_max = getMmaxBending(JRF, W_su)
    M_max = JRF*W_su;
end

function I_allowed = getIAllowedBending(M_max, W_su, sigma_allowed)
    I_allowed = M_max*(W_su/2)/sigma_allowed;
end

function legDim = getLegDimsBending(I_allowed, W_su)
    syms a real positive
    b = FrameAnalysis.ratio*a;
    I1 = (a*b^3)/12;
    A1 = a*b;
    d1 = (W_su-b)/2;
    I2 = (b*(a-b)^3)/12;
    A2 = b*(a-b);
    d2 = W_su/2 - b - (a-b)/2;

    I = 4*(I1 + A1*d1^2) + 4*(I2 + A2*d2^2);
    eqn = I == I_allowed;
    a_sol = vpasolve(eqn, a, I_allowed^(1/4));
    a_sol = double(a_sol);
    a_sol = a_sol(a_sol > 0 & imag(a_sol) == 0);
    a_sol = min(a_sol);
    b_sol = FrameAnalysis.ratio*a_sol;

    legDim.a = a_sol;
    legDim.b = b_sol;
end

% REQUIRED LEG DIMENSIONS
function legDimensions = getLegDimensions(n, JRF, E, Sy, Ssy, d_isu)
    % Calculate geometry
    H_su = FrameAnalysis.getHsu(d_isu);
    W_su = FrameAnalysis.wsufunction(d_isu, H_su);

    % Buckling
    Pcr1 = FrameAnalysis.getPcrfr(JRF, n);
    I_allowed1 = FrameAnalysis.getIAllowed(Pcr1, E);
    Dim1 = FrameAnalysis.getLegDimsBuckling(I_allowed1);
    % Shear
    tau_allowed2 = FrameAnalysis.getTau(Ssy, n);
    Dim2 = FrameAnalysis.getLegDimsShear(JRF, tau_allowed2);
    % Bending
    M_max3 = FrameAnalysis.getMmaxBending(JRF, W_su);
    sigma_allowed3 = FrameAnalysis.getSigma(Sy, n);
    I_allowed3 = FrameAnalysis.getIAllowedBending(M_max3, W_su, sigma_allowed3);
    Dim3 = FrameAnalysis.getLegDimsBending(I_allowed3, W_su);

    legDimensions.a = max([Dim1.a, Dim2.a, Dim3.a]);
    legDimensions.b = max([Dim1.b, Dim2.b, Dim3.b]);
end

% -------- LIP --------

function F_lp = getFlp(M_k, F_k)
    F_lp = M_k/FrameAnalysis.r_ball + F_k/2;  % part of the moment and the JRF in worst case scenario
end

% COMPRESSION
function t_lp = getLipThicknessComp(F_lp, sigma_allowed, w_lp)
    t_lp = F_lp/(sigma_allowed*w_lp);
end

end
end
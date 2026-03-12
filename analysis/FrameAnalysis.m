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

% Projected height of the support plate along the inclined leg axis.
% Derived from the radial gap between the ball outer radius and the shaft OD,
% projected onto the bisector of the leg opening angle phi1.
function H_su = getHsu(d_isu, r_b)
    H_su = ((r_b-d_isu)/2)*cos(FrameAnalysis.phi1/2);
end

% Minimum support plate thickness from Euler column buckling.
% Fixed-free column:  Pcr = pi^2 * E * I / (4 * H_su^2)
% With I = d_isu * t^3 / 12 (rectangular cross-section), solved for t:
%   t = (3 * Pcr * H_su^2 / (pi^2 * E * d_isu))^(1/3)
function t_su = gettsuBuckling(log, n, JRF, E, d_isu, r_b)
    Pcr = FrameAnalysis.getPcrsu(JRF, n);
    fprintf(log, 'Pcr = %.2f N\n', Pcr);
    H_su = FrameAnalysis.getHsu(d_isu, r_b);
    fprintf(log, 'H_su = %.2f mm\n', H_su*1000);
    t_su = (3*Pcr*H_su^2/(pi()^2*E*d_isu))^(1/3);
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
    t_su = 6*JRF*H_su/(sigma*d_isu^2);
end

function t_su = gettsuBendingYZ(JRF, H_su, d_isu, sigma)
    JRF_z = JRF*0.25;
    t_su = sqrt(6*JRF_z*H_su/(sigma*d_isu));
end

% REQUIRED SUPPORT THICKNESS

function t_su = getSupportThickness(log, n, JRF, E, Ssy, Sy, d_isu, r_b)

    fprintf(log, 'Limiting Support Thicknesses:\n');
    % Buckling
    fprintf(log, 'Buckling\n');
    t1 = FrameAnalysis.gettsuBuckling(log, n, JRF, E, d_isu, r_b);
    fprintf(log, 't_su (buckling): %.2f mm\n', t1*1000);
    % Shear
    fprintf(log, 'Shear\n');
    tau = FrameAnalysis.getTau(Ssy, n);
    fprintf(log, 'tau_allowed: %.2f Pa\n', tau);
    t2 = FrameAnalysis.gettsuShearXY(JRF, d_isu, tau);
    fprintf(log, 't_su (shear): %.2f mm\n', t2*1000);
    % Bending
    fprintf(log, 'Bending\n');
    sigma = FrameAnalysis.getSigma(Sy, n);
    fprintf(log, 'sigma_allowed: %.2f Pa\n', sigma);
    t3 = FrameAnalysis.gettsuBendingXY(JRF, FrameAnalysis.getHsu(d_isu, r_b), d_isu, sigma);
    t4 = FrameAnalysis.gettsuBendingYZ(JRF, FrameAnalysis.getHsu(d_isu, r_b), d_isu, sigma);
    fprintf(log, 't_su (bending XY): %.2f mm\n', t3*1000);
    fprintf(log, 't_su (bending YZ): %.2f mm\n\n', t4*1000);

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

% Minimum leg cross-section outer dimension from Euler column buckling.
% Cross-section is a C-channel: outer dimension a, wall thickness b = ratio*a.
% The centroid and second moment of area are computed via the parallel-axis theorem,
% then vpasolve finds the 'a' that achieves exactly I_allowed.
function legDim = getLegDimsBuckling(I_allowed)
    % Since b = ratio*a, every area term scales as a^2 and every second moment as a^4.
    % Evaluating I at a=1 gives the constant I_unit, then a = (I_allowed/I_unit)^0.25 exactly.
    r   = FrameAnalysis.ratio;
    b1  = r;
    cy1 = (1 - b1^2 + b1) / (4 - 2*b1);
    I1u = b1^3 / 12;
    A1u = b1;          d1u = cy1 - b1/2;
    I2u = b1*(1-b1)^3 / 12;
    A2u = b1*(1-b1);   d2u = b1 + (1-b1)/2 - cy1;
    I_unit = I1u + I2u + A1u*d1u^2 + A2u*d2u^2;  % I(a=1)

    a_sol = (I_allowed / I_unit)^0.25;
    legDim.a = a_sol;
    legDim.b = r * a_sol;
end
    
% SHEAR
% Minimum leg outer dimension from transverse shear stress in the hollow rectangular section.
% tau = 3*V / (8 * A_web)  where A_web = a^2 - (a-b)^2 (area difference outer vs inner).
% Each of the 4 legs carries JRF/4; vpasolve finds 'a' satisfying tau = tau_allowed.
function legDim = getLegDimsShear(JRF, tau_allowed)
    % With b = ratio*a: a^2 - (a-b)^2 = a^2*ratio*(2-ratio), solve directly for a.
    r     = FrameAnalysis.ratio;
    a_sol = sqrt(3*JRF / (8 * tau_allowed * r * (2 - r)));
    legDim.a = a_sol;
    legDim.b = r * a_sol;
end

% BENDING
function M_max = getMmaxBending(JRF, W_su)
    M_max = JRF*W_su;
end

function I_allowed = getIAllowedBending(M_max, W_su, sigma_allowed)
    I_allowed = M_max*(W_su/2)/sigma_allowed;
end

% Minimum leg outer dimension from bending stress in the frame legs.
% The 4 legs are arranged symmetrically; each is a thin-walled C-channel.
% Total I about the bending axis is computed using the parallel-axis theorem for
% each leg's flange and web, offset from the neutral axis by their respective distances.
% vpasolve finds the 'a' such that the combined I equals I_allowed.
function legDim = getLegDimsBending(I_allowed, W_su)
    % W_su introduces cross terms between the constant W_su and unknown a,
    % so the pure a^4 scaling trick does not apply. fzero() (base MATLAB,
    % no toolbox) finds the root of I(a) - I_allowed on [1e-5, 1] m.
    r = FrameAnalysis.ratio;
    Ifunc = @(a) 4*( (a*(r*a)^3)/12    + (a*(r*a))   *((W_su - r*a)/2)^2 ) + ...
                 4*( ((r*a)*(a-r*a)^3)/12 + ((r*a)*(a-r*a))*(W_su/2 - r*a - (a-r*a)/2)^2 );
    a_sol = fzero(@(a) Ifunc(a) - I_allowed, [1e-5, 1]);
    legDim.a = a_sol;
    legDim.b = r * a_sol;
end

% REQUIRED LEG DIMENSIONS
function legDimensions = getLegDimensions(log, n, JRF, E, Sy, Ssy, d_isu, r_b)

    fprintf(log, 'Limiting Leg Dimensions:\n');
    % Calculate geometry
    H_su = FrameAnalysis.getHsu(d_isu, r_b);
    W_su = FrameAnalysis.wsufunction(d_isu, H_su);

    % Buckling
    fprintf(log, 'Buckling\n');
    Pcr1 = FrameAnalysis.getPcrfr(JRF, n);
    I_allowed1 = FrameAnalysis.getIAllowed(Pcr1, E);
    Dim1 = FrameAnalysis.getLegDimsBuckling(I_allowed1);
    fprintf(log, 'Pcr: %.2f N\n', Pcr1);
    fprintf(log, 'I_allowed: %.2f mm^4\n', I_allowed1*1000000000000);
    fprintf(log, 'a: %.2f mm\n', Dim1.a*1000);
    fprintf(log, 'b: %.2f mm\n', Dim1.b*1000);
    % Shear
    fprintf(log, 'Shear\n');
    tau_allowed2 = FrameAnalysis.getTau(Ssy, n);
    Dim2 = FrameAnalysis.getLegDimsShear(JRF, tau_allowed2);
    fprintf(log, 'tau_allowed: %.2f Pa\n', tau_allowed2);
    fprintf(log, 'a: %.2f mm\n', Dim2.a*1000);
    fprintf(log, 'b: %.2f mm\n', Dim2.b*1000);
    % Bending
    fprintf(log, 'Bending\n');
    M_max3 = FrameAnalysis.getMmaxBending(JRF, W_su);
    sigma_allowed3 = FrameAnalysis.getSigma(Sy, n);
    I_allowed3 = FrameAnalysis.getIAllowedBending(M_max3, W_su, sigma_allowed3);
    Dim3 = FrameAnalysis.getLegDimsBending(I_allowed3, W_su);
    fprintf(log, 'sigma_allowed: %.2f Pa\n', sigma_allowed3);
    fprintf(log, 'I_allowed: %.2f mm^4\n', I_allowed3*1000000000000);
    fprintf(log, 'a: %.2f mm\n', Dim3.a*1000);
    fprintf(log, 'b: %.2f mm\n', Dim3.b*1000);

    legDimensions.a = max([Dim1.a, Dim2.a, Dim3.a]);
    legDimensions.b = max([Dim1.b, Dim2.b, Dim3.b]);
end

% -------- LIP --------

% Equivalent force the lip must resist in the worst case (heel strike).
% The knee moment creates a force couple at the ball radius (M_k / r_b),
% and half the vertical JRF acts directly on the lip in compression.
function F_lp = getFlp(M_k, F_k, r_b)
    F_lp = M_k/r_b + F_k/2;  % moment contribution + direct JRF component
end

% COMPRESSION
% Minimum lip thickness from compressive stress:  t = F_lp / (sigma_allowed * w_lp)
function t_lp = getLipThicknessComp(F_lp, sigma_allowed, w_lp)
    t_lp = F_lp/(sigma_allowed*w_lp);
end

end
end
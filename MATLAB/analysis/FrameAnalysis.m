% =========================================================
% FRAME ANALYSIS
% =========================================================

%{

% =========================================================
% SUBSCRIPT CONVENTION
% =========================================================

s : shaft
su : supports
fr: frame
lp : lip
b : ball

%}


classdef FrameAnalysis

properties (Constant)

    phi1 = deg2rad(100);   % [rad], angle between the front legs and back legs
    phi2 = deg2rad(55); % [rad], angle between front leg and the lip
    a = 0.015;  % [m], width of the legs set to 15mm

end

methods (Static)

function tau_allowed = getTau(Ssy, n)
    tau_allowed = Ssy/n;
end

function sigma_allowed = getSigma(Sy, n)
    sigma_allowed = Sy/n;
end

% =========================================================
% SUPPORTS
% =========================================================

% BUCKLING
function Pcr = getPcrsu(JRF, n)
    Pcr = (JRF/2)*n;
end

% Projected height of the support plate along the inclined leg axis.
% Derived from the radial gap between the ball outer radius and the shaft OD,
% projected onto the bisector of the leg opening angle phi1.
function H_su = getHsu(d_isu, r_b)
    H_su = (r_b-(d_isu/2))*cos(FrameAnalysis.phi1/2);
end

% Minimum support plate thickness from Euler column buckling.
% Fixed-fixed column:  Pcr = pi^2 * E * I / (0.5 * H_su)^2
% With I = d_isu * t_su^3 / 12 (rectangular cross-section), solved for t_su:
%   t_su = (3 * Pcr * H_su^2 / (pi^2 * E * d_isu))^(1/3)
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
    t_su = sqrt(6*(JRF/2)*H_su/(sigma*d_isu));
end

% REQUIRED SUPPORT THICKNESS

function t_su = getSupportThickness(log, n, JRF, E, Ssy, Sy, d_isu, r_b)

    fprintf(log, 'Limiting Support Thicknesses for n = %.2f:\n', n);
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

% =========================================================
% LEGS
% =========================================================

% BUCKLING
function Pcr = getPcrfr(JRF, n)
    Pcr = n*(JRF/4);
end

function I_allowed = getIAllowed(Pcr, E, L_fr)
    I_allowed = Pcr*L_fr^2/(4*E*pi()^2);
end

% Function for I (buckling)
% The centroid and second moment of area are computed via the parallel-axis theorem
function I = computeI_buckling(b, a)
    cy = (a^2 - b.^2 + a.*b) ./ (4*a - 2*b);

    I1 = (1/12)*a.*b.^3;
    A1 = a.*b;
    d1 = cy - b/2;

    I2 = (1/12)*b.*(a-b).^3;
    A2 = b.*(a-b);
    d2 = b + (a-b)/2 - cy;

    I = (I1 + A1.*d1.^2) + (I2 + A2.*d2.^2);
end

% Minimum leg cross-section outer dimension from Euler column buckling.
% Cross-section is an L-shaped cross-section: outer dimension a, wall thickness b
% a is set to 15mm, solve for b
function legDim = getLegDimsBuckling(I_allowed)
    a = FrameAnalysis.a;

    f = @(b) FrameAnalysis.computeI_buckling(b, a) - I_allowed;

    % 0 < b < a
    b_vals = linspace(1e-6, a-1e-6, 1000);
    f_vals = arrayfun(f, b_vals);

    % Find first sign change
    idx = find(f_vals(1:end-1).*f_vals(2:end) < 0, 1);

    if isempty(idx)
        error('No valid solution found for b in (0, a)');
    end

    % Solve in that small interval
    b_sol = fzero(f, [b_vals(idx), b_vals(idx+1)]);

    legDim.b = double(b_sol);
end
    
% SHEAR
function legDim = getLegDimsShear(JRF, tau_allowed)
    a = FrameAnalysis.a;
    b = a - sqrt(a^2 - (3/8)*(JRF/tau_allowed));
    legDim.b = b;
end

% BENDING
function M_max = getMmaxBending(JRF, L_fr)
    M_max = JRF*L_fr;
end

function I_allowed = getIAllowedBending(M_max, W_su, sigma_allowed)
    I_allowed = M_max*(W_su/2)/sigma_allowed;
end

% Function for I (bending)
% The centroid and second moment of area are computed via the parallel-axis theorem
function I = computeI_bending(b, a, W_su)
    I1 = (1/12)*a.*b.^3;
    A1 = a.*b;
    d1 = W_su/2 - b/2;

    I2 = (1/12)*b.*(a-b).^3;
    A2 = b.*(a-b);
    d2 = W_su/2 - b - (a-b)/2;

    I = 4*(I1 + A1.*d1.^2) + 4*(I2 + A2.*d2.^2);
end

% Minimum leg outer dimension from bending stress in the frame legs.
% The 4 legs are arranged symmetrically; each is a thin-walled L-shape.
% Total I about the bending axis is computed using the parallel-axis theorem for
% each leg's flange and web, offset from the neutral axis by their respective distances.
% vpasolve finds the 'b' such that the combined I equals I_allowed.
function legDim = getLegDimsBending(I_allowed, W_su)
    a = FrameAnalysis.a;

    f = @(b) FrameAnalysis.computeI_bending(b, a, W_su) - I_allowed;

    % 0 < b < a
    b_vals = linspace(1e-6, a-1e-6, 1000);
    f_vals = arrayfun(f, b_vals);

    % Find first sign change
    idx = find(f_vals(1:end-1).*f_vals(2:end) < 0, 1);

    if isempty(idx)
        error('No valid solution found for b in (0, a)');
    end

    % Solve in that small interval
    b_sol = fzero(f, [b_vals(idx), b_vals(idx+1)]);

    legDim.b = double(b_sol);
end

% REQUIRED LEG DIMENSIONS
function legDimensions = getLegDimensions(log, n, JRF, E, Sy, Ssy, d_isu, r_b, L_fr)

    fprintf(log, 'Limiting Leg Dimensions for n = %.2f:\n', n);

    legDimensions.a = FrameAnalysis.a;
    fprintf(log, 'For a leg width, a = %.2f mm\n', legDimensions.a*1000);
    % Calculate geometry
    H_su = FrameAnalysis.getHsu(d_isu, r_b);
    W_su = FrameAnalysis.wsufunction(d_isu, H_su);

    % Buckling
    fprintf(log, 'Buckling\n');
    Pcr1 = FrameAnalysis.getPcrfr(JRF, n);
    I_allowed1 = FrameAnalysis.getIAllowed(Pcr1, E, L_fr);
    Dim1 = FrameAnalysis.getLegDimsBuckling(I_allowed1);
    fprintf(log, 'Pcr: %.2f N\n', Pcr1);
    fprintf(log, 'I_allowed: %.2f mm^4\n', I_allowed1*1000000000000);
    fprintf(log, 'b: %.6f mm\n', Dim1.b*1000);
    % Shear
    fprintf(log, 'Shear\n');
    tau_allowed2 = FrameAnalysis.getTau(Ssy, n);
    Dim2 = FrameAnalysis.getLegDimsShear(JRF, tau_allowed2);
    fprintf(log, 'tau_allowed: %.2f Pa\n', tau_allowed2);
    fprintf(log, 'b: %.6f mm\n', Dim2.b*1000);
    % Bending
    fprintf(log, 'Bending\n');
    M_max3 = FrameAnalysis.getMmaxBending(JRF, L_fr);
    sigma_allowed3 = FrameAnalysis.getSigma(Sy, n);
    I_allowed3 = FrameAnalysis.getIAllowedBending(M_max3, W_su, sigma_allowed3);
    Dim3 = FrameAnalysis.getLegDimsBending(I_allowed3, W_su);
    fprintf(log, 'sigma_allowed: %.2f Pa\n', sigma_allowed3);
    fprintf(log, 'I_allowed: %.2f mm^4\n', I_allowed3*1000000000000);
    fprintf(log, 'b: %.6f mm\n', Dim3.b*1000);

    limiting_b = max([Dim1.b, Dim2.b, Dim3.b]);
    fprintf(log, 'Final limiting b: %.6f mm\n', limiting_b*1000);

    legDimensions.b = limiting_b;
end

% =========================================================
% LIP
% =========================================================

% Equivalent force the lip must resist in the worst case (heel strike).
% The knee moment creates a force couple at the ball radius (M_k / r_b),
% and the vertical JRF acts directly on the lip in compression.
function F_lp = getFlp(M_k, F_k, r_b)
    F_lp = M_k/r_b + F_k;  % moment contribution + direct JRF component
end

% COMPRESSION
% Minimum lip thickness from compressive stress:  t = F_lp / (sigma_allowed * w_lp)
function t_lp = getLipThicknessComp(log, F_lp, sigma_allowed, w_lp, n)
    t_lp = F_lp/(sigma_allowed*w_lp);

    fprintf(log, '\nLimiting Lip Thickness for n = %.2f:\n', n);
    fprintf(log, 'F_lp: %.2f N\n', F_lp);
    fprintf(log, 't_lp: %.2f mm\n\n', t_lp*1000);
end

% =========================================================
% FINAL FRAME THICKNESS
% =========================================================

% To simplify the mold geometry and avoid any extremely thin components
% (difficult to demold), the same thickness for the supports, legs, and
% lip will be used

function frame_thickness = t_frame(log, t_su, b, t_lp)
    limiting_frame_thickness = max([t_su, b, t_lp]);
    frame_thickness = ceil(limiting_frame_thickness*1000)/1000;   % round up to the nearest frame thickness

    fprintf(log, 'Final frame thickness:\nt_frame = %.2f mm\n\n', frame_thickness*1000);
end

end
end
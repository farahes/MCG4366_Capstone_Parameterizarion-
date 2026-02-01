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
    r_ball = 0.04; % [m], currently arbitrary
    phi1 = 2*pi()/3;   % [rad], angle between the front legs and back legs
    L_fr = 0.12; % [m], length of the legs, currently arbitrary
end

methods (Static)

% -------- SUPPORTS --------
% flanges that support the shaft

% BUCKLING

function Pcr = getPcrsu(JRF, n)
    Pcr = (JRF/2)*n;
end

% maximum thickness required to prevent buckling of the supports
function t_su = gettsuBuckling(n, JRF, L, E, d)
    Pcr = FrameAnalysis.getPcrsu(JRF, n);
    t_su = (12*Pcr*L^2/(pi()^2*E*d))^(1/3);
end

% SHEAR (in both directions)

% BENDING

% REQUIRED SUPPORT THICKNESS

function t_su = getSupportThickness(n, JRF, L, E, d)

    % Buckling
    t1 = FrameAnalysis.gettsuBuckling(n, JRF, L, E, d);
    % Shear

    % Bending

    t_su = max([t1]);
end

% -------- LEGS --------
% four main columns making up the height of the prosthesis

% BUCKLING

% Effective length of the columns
function Le = getLe(L)
    Le = 0.5*L; % fixed-fixed end condition
end

function Pcr = getPcr(JRF)  % takes the JRF along the axis of the legs
    Pcr = JRF/4; % should increase the division so theres some distance between P and Pcr
end

function legDimensions = getLegDimensionsBuckling(JRF, L, E)
    ratio = 0.3;    % ratio of b to a
    Le = FrameAnalysis.getLe(L);
    Pcr = FrameAnalysis.getPcr(JRF);
    I_req = Pcr*Le^2/(pi()^2*E);
    cy = @(a,b) (b*(a-b)*(b + (a-b)/2) + a*b*(b/2))/(b*(a-b) + a*b);

    % moment of inertia
    I1 = @(a,b) (b*(a-b)^3)/12;
    A1 = @(a,b) a*(a-b);
    d1 = @(a,b) b + (a-b)/2 -cy(a,b);
    I2 = @(a,b) (a*b^3)/12;
    A2 = @(a,b) a*b;
    d2 = @(a,b) b/2 - cy(a,b);
    I = @(a,b) (I1(a,b) + A1(a,b)*d1(a,b)^2) + (I1(a,b) + A1(a,b)*d1(a,b)^2);
    
    func = @(a) I(a, ratio*a) - I_req;
    a0 = 0.01; % initial guess of 1 cm
    a = fsolve(func, a0);
    b = ratio*a;

    % return a and b inside the struct
    legDimensions.a = a;
    legDimensions.b = b;
end

% BENDING
% component of the JRF perpendicular to the legs causes a bending moment on
% the supports

% REQUIRE LEG DIMENSIONS
function legDimensions = getLegDimensions(JRF, L, E)

    % Buckling
    Dimensions1 = FrameAnalysis.getLegDimensionsBuckling(JRF, L, E);
    % Shear

    % Bending

    legDimensions.a = max([Dimensions1.a]);
    legDimensions.b = max([Dimensions1.b]);
end

% -------- LIP --------
% prevent the hinge from over extension

% BUCKLING

% SHEAR

% BENDING


end
end
% ============================================================
% STRUCTURAL ANALYSIS OF PROSTHETIC PYRAMID ADAPTER
% FINAL VERSION (SUBMISSION READY)
%
% Governing case : Heel strike
% Material : Ti-6Al-4V
% Method : Mechanics-based hand calculations
% ============================================================

classdef PyramidAdapter

properties (Constant)

    % GEOMETRY (4R23-BASED IDEALISATION)
    l = 19.0; % Effective cantilever length [mm]
    D = 24.0; % Effective beam depth [mm]
    T = 5.0; % Beam thickness [mm]
    e_upper = 15.0; % Upper adapter eccentricity [mm]
    e_bottom = 25.0; % Bottom adapter eccentricity [mm]
    alpha = 0.141; % Torsion coefficient (rectangular section)
    
    % TRANSVERSE OFFSET
    r_t_upper = 7.0; % Upper transverse offset [mm]
    r_t_bottom = 12.0; % Bottom transverse offset [mm]

end

methods (Static)

% Cross-sectional area of the rectangular beam section (D = depth, T = thickness) [mm^2]
function A = getA(D, T)
    A = D * T;
end

% Second moment of area about the bending axis [mm^4].
% T is the thin dimension (in the bending direction):  I = D*T^3 / 12
function I = getI(D, T)
    I = (D * T^3) / 12;
end

% Distance from neutral axis to extreme fibre = T/2 (symmetric rectangular section) [mm]
function c = getc(T)
    c = T / 2;
end

% Torsional moment on each adapter from the vertical load Fy acting at a transverse offset.
% T_tors = Fy * r_t  (moment arm from load line to section centroid in the z-direction)
function Ttors = getTorsion(Fy)
    Ttors.upper = Fy*PyramidAdapter.r_t_upper;  % [N·mm]
    Ttors.lower = Fy*PyramidAdapter.r_t_bottom; % [N·mm]
end

% Upper pyramid adapter yield safety factor using the von Mises criterion.
% Fy (vertical) causes: axial stress, bending via eccentricity e_upper,
%                       and torsion via transverse offset r_t_upper.
% Fx (horizontal) causes: direct shear (Anterior-Posterior direction).
% FoS = Sy / sigma_vm  where  sigma_vm = sqrt(sigma_n^2 + 3*(tau_AP^2 + tau_t^2))
function FoS_yield_u = getFoS_yield_u(Fx, Fy, Sy)
    % Bending moment from vertical load acting at eccentricity
    M_u = Fy * PyramidAdapter.e_upper; % [N·mm]

    % Beam force per web
    Fbeam_u = (Fy / 2) + (M_u / (2 * PyramidAdapter.l));

    % Normal stress
    sigma_ax_u = Fy / PyramidAdapter.getA(PyramidAdapter.D, PyramidAdapter.T);
    sigma_b_u = (Fbeam_u * PyramidAdapter.l * PyramidAdapter.getc(PyramidAdapter.T)) / PyramidAdapter.getI(PyramidAdapter.D, PyramidAdapter.T);
    sigma_n_u = sigma_ax_u + sigma_b_u;

    % Shear stresses
    tau_AP_u = 1.5 * Fx / PyramidAdapter.getA(PyramidAdapter.D, PyramidAdapter.T);
    tau_t_u = PyramidAdapter.getTorsion(Fy).upper / (PyramidAdapter.alpha * PyramidAdapter.D * PyramidAdapter.T^2);

    % von Mises stress
    sigma_vm_u = sqrt( sigma_n_u^2 + 3 * (tau_AP_u^2 + tau_t_u^2) );
    FoS_yield_u = Sy / sigma_vm_u;
end

% Lower pyramid adapter yield safety factor (same method as upper adapter).
% Uses e_bottom and r_t_bottom, which are larger than the upper adapter values,
% resulting in a more conservative (lower) safety factor for the lower interface.
function FoS_yield_b = getFoS_yield_b(Fx, Fy, Sy)
    M_b = Fy * PyramidAdapter.e_bottom; % [N·mm]

    Fbeam_b = (Fy / 2) + (M_b / (2 * PyramidAdapter.l));

    sigma_ax_b = Fy / PyramidAdapter.getA(PyramidAdapter.D, PyramidAdapter.T);
    sigma_b_b = (Fbeam_b * PyramidAdapter.l * PyramidAdapter.getc(PyramidAdapter.T)) / PyramidAdapter.getI(PyramidAdapter.D, PyramidAdapter.T);
    sigma_n_b = sigma_ax_b + sigma_b_b;

    tau_AP_b = 1.5 * Fx / PyramidAdapter.getA(PyramidAdapter.D, PyramidAdapter.T);
    tau_t_b = PyramidAdapter.getTorsion(Fy).lower / (PyramidAdapter.alpha * PyramidAdapter.D * PyramidAdapter.T^2);

    sigma_vm_b = sqrt( sigma_n_b^2 + 3 * (tau_AP_b^2 + tau_t_b^2) );
    FoS_yield_b = Sy / sigma_vm_b;
end

end
end

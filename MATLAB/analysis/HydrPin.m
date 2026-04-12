% =========================================================
% HYDRAULIC PIN PARAMETERIZATION
% MCG4366 Group 12
%
% Design analysis:
% - Static shear safety factor
% - Static bending safety factor
% =========================================================

classdef HydrPin

properties (Constant)

    n = 2;

    % Material properties (AISI 304 SS)
    Su = 505e6; % Ultimate tensile strength [Pa]
    Sy = 215e6; % Yield strength [Pa]

    % Fatigue factors (Juvinall)
    CS = 0.85; % Surface factor
    CG = 0.9; % Gradient factor
    CR = 0.81; % Reliability factor

    % Pin geometry
    r_percent = 0.8;    % percentage of the ball radius where the pin is mounted

end

methods (Static)

    % Shear yield stress
    function Ssy = Ssy()
        Ssy = 0.577 * HydrPin.Sy;
    end

    % Fatigue parameters
    function Se_prime = Se_prime()
        Se_prime = 0.5*HydrPin.Su;
    end

    function Se = Se()
        Se = HydrPin.Se_prime()*HydrPin.CS*HydrPin.CG*HydrPin.CR;
    end

    % Converts the knee joint moment to a shear force acting on the pin.
    % The pin is mounted at r_percent * r_ball from the ball centre,
    % so the force is:  F = M / (r_percent * r_ball)  (moment-arm conversion).
    function F_max = F_max(M, r_ball)
        F_max = M/(HydrPin.r_percent*r_ball);
    end

    % Maximum bending stress in a solid circular cross-section:  σ = M*c / I
    % c = d/2 (distance to outer fibre),  I = pi*d^4/64 (2nd moment of area).
    function sigma_bending = BendingStress(M, d)
        sigma_bending = M*(d/2)/(pi*d^4/64);
    end


% =========================================================
%  MAIN HYDRAULIC PIN DIMENSIONS
% =========================================================

    % Iteratively sizes the hydraulic pin diameter to satisfy bending and shear safety factors.
    %
    % Two pins are designed:
    %   Upper pin: spans the full ball width (w_ball). The hydraulic cylinder sits in the
    %              centre (length b_upper), with overhangs a_upper on each side.
    %   Lower pin: shorter, braced by two frame pads (a_lower fixed conservatively at 7 mm).
    %
    % The force F is multiplied by 1.5x to cover higher load cases.
    % Pin diameter starts at 10 mm and is incremented by 1 mm until all FoS >= n.
    function PinDim = PinDim(M_k, w_ball, r_ball, d_hydraulic)

        % Convert knee moment to pin shear force; 1.5x dynamic load factor
        F = HydrPin.F_max(M_k, r_ball)*1.5;

        % -------- UPPER PIN DIMENSIONS --------
        % the upper pin will be the the length of the ball thickness
        % (see drawings)

        b_upper = (d_hydraulic + 0.004);    % 2 mm buffer of space on both sides of the hydraulic
        a_upper = (w_ball - b_upper)/2;

        M_upper = (F/2)*((a_upper/2) + (b_upper/2));

        % -------- LOWER PIN DIMENSIONS --------
        % the lower pin will be supported by two supports on the base of
        % the frame

        b_lower = (0.75*d_hydraulic + 0.004);    % 2 mm buffer of space on both sides of the hydraulic
        a_lower = 0.007;    % set at 7 mm (CHANGEABLE)

        M_lower = (F/2)*((a_lower/2) + (b_lower/2));

        % initial guess for pin diameter [m]
        d_pin = 10e-3; 

        % Iterate over pin sizes until safety factors are met
        n_bending_upper = 0;
        n_bending_lower = 0;
        n_shear = 0;
        while (n_bending_upper < HydrPin.n) || (n_bending_lower < HydrPin.n) || (n_shear < HydrPin.n)

            % -------- UPPER PIN STRESSES --------
            sigma_max_upper = HydrPin.BendingStress(M_upper, d_pin);    % maximum bending stress at edges
            n_bending_upper = HydrPin.Sy/sigma_max_upper;

            % -------- LOWER PIN STRESSES --------
            sigma_max_lower = HydrPin.BendingStress(M_lower, d_pin);    % maximum bending stress at edges
            n_bending_lower = HydrPin.Sy/sigma_max_lower;

            tau_shear_NA = (4/3)*(F/2)/(pi*d_pin^2/4);  % shear stress at the neutral axis
            tau_shear_edge_upper = sigma_max_upper/2;   % shear stress at the edge
            tau_shear_edge_lower = sigma_max_lower/2;   % shear stress at the edge
            tau_max = max([tau_shear_NA, tau_shear_edge_upper, tau_shear_edge_lower]);
            n_shear = HydrPin.Ssy()/tau_max;

            % Test failure by fatigue (NOT FINISHED FOR PARAMETERIZATION)

            if (n_bending_upper < HydrPin.n) || (n_bending_lower < HydrPin.n) || (n_shear < HydrPin.n)
                d_pin = d_pin + 0.001; % increment diameter of the pin by 1 mm
            end
        end

        % Return final pin dimensions
        PinDim.length_upper = w_ball;
        PinDim.length_lower = b_lower + 2*a_lower;
        PinDim.diameter = d_pin;

    end

end

end
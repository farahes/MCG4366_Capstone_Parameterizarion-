classdef HydraulicNeedleValve

properties (Constant)
    DeltaP = 6e6;                 % Pressure drop [Pa]
    rho = 963;                    % Density [kg/m^3]
    Cd = 0.65;                    % Discharge coefficient
    Q_max = 78.5e-6;            % Max flow rate [m^3/s]
end

methods (Static)

% Orifice diameter sweep from 0.12 mm to 1.75 mm (feasible needle valve range).
% Returns values in metres for use in the orifice flow equation.
function d = getDrange()
    d_mm = linspace(0.12, 1.75, 1000);   % Diameter [mm]
    d = d_mm * 1e-3;                     % Convert to meters
end

% Orifice flow equation (Bernoulli-based):  Q = Cd * A * sqrt(2*ΔP / ρ)
% Cd accounts for viscous losses and vena contracta contraction at the orifice.
% ΔP is the pressure drop across the valve; ρ is the hydraulic fluid density.
function Q = getFlowRate(d, Cd, DeltaP, rho)
    A = pi .* d.^2 ./ 4;                 % Orifice cross-sectional area [m^2]
    Q = Cd .* A .* sqrt(2*DeltaP/rho);   % Volume flow rate [m^3/s]
end

% Selects the optimal needle valve orifice diameter.
% Strategy: find the largest diameter whose flow rate stays within Q_max.
% A larger orifice gives more controllable damping (less pressure-sensitive).
% Also computes the Cv flow coefficient for standard valve procurement.
function results = getValveSize()
    d = HydraulicNeedleValve.getDrange();
    Q = HydraulicNeedleValve.getFlowRate(d, HydraulicNeedleValve.Cd, HydraulicNeedleValve.DeltaP, HydraulicNeedleValve.rho);

    % Keep only diameters where Q does not exceed the maximum allowable flow
    valid_idx = Q <= HydraulicNeedleValve.Q_max;
    d_valid = d(valid_idx);
    Q_valid = Q(valid_idx);

    % Best diameter (max flow without exceeding limit)
    [d_best, idx] = max(d_valid);
    Q_best = Q_valid(idx);

    % Cv calculation
    Q_gpm = Q_best * 15850.3;        % m^3/s → gpm
    DeltaP_psi = HydraulicNeedleValve.DeltaP / 6894.76;   % Pa → psi
    SG = HydraulicNeedleValve.rho / 1000;
    Cv = Q_gpm * sqrt(SG / DeltaP_psi);

    % results
    results.d_best_mm = d_best * 1e3;
    results.Q_best    = Q_best;
    results.Q_Lmin    = Q_best * 60 * 1000;
    results.Cv        = Cv;
    results.d         = d;
    results.Q         = Q;
end

% Plot needle valve flow rate vs diameter
function plot(results)
    d_mm = results.d * 1e3;
    Q_Lmin = results.Q * 60 * 1000;

    Q_max_Lmin = HydraulicNeedleValve.Q_max * 60 * 1000;
    figure;
    plot(d_mm, Q_Lmin, 'LineWidth', 1.5);
    hold on;
    yline(Q_max_Lmin, '--r', 'Q_{max}');
    xlabel('Diameter [mm]');
    ylabel('Flow rate [L/min]');
    title('Needle Valve Flow vs Diameter');
    grid on;
end

end
end

% ============================================================
% STRUCTURAL ANALYSIS OF PROSTHETIC PYRAMID ADAPTER
% UPPER AND LOWER ADAPTER ANALYSIS
%
% Governing case : Heel strike (Fy = 2060.1 N, Fx = 137.3 N)
% Material : AISI 316L Stainless Steel
% Method : Mechanics-based hand calculations with thread & bolt analysis
% ============================================================

classdef PyramidAdapter

properties (Constant)

    % ========== MATERIAL PROPERTIES (AISI 316L STAINLESS STEEL) ==========
    sigma_y = 170;      % Yield Tensile Strength [MPa]
    sigma_u = 485;      % Ultimate Tensile Strength [MPa]
    tau_y = 98.6;       % Shear Yield Strength (0.58*sigma_y) [MPa]
    E = 193;            % Young's Modulus [GPa]
    nu = 0.27;          % Poisson's Ratio
    rho = 8.0;          % Density [g/cm³]
    S_e = 160;          % Endurance Limit [MPa]
    
    % ========== DESIGN LOADS (at Heel Strike) ==========
    F_v = 2060.1;       % Vertical load [N]
    F_AP = 137.3;       % Anterior-Posterior shear load [N]
    F_res = 2064.7;     % Resultant force [N]
    n_safety = 2;       % Design safety factor
    
    % ========== UPPER PYRAMID ADAPTER GEOMETRY ==========
    % Thread dimensions
    d_thread = 30;      % Thread major diameter [mm]
    p_thread = 1.50;    % Thread pitch [mm]
    L_e = 12;           % Thread engagement depth [mm]
    n_e = 8;            % Number of engaged threads
    A_s_per = 106.03;   % Shear area per thread [mm²]
    A_s_total = 848.23; % Total shear area (8 threads) [mm²]
    
    % Pyramid boss dimensions
    b_pyr_upper = 12.23;  % Pyramid boss side length (solid square) [mm]
    A_pyr_upper = 149.57; % Cross-sectional area of pyramid boss [mm²]
    h_boss_upper = 6.11;  % Boss height [mm]
    e_upper = 1.51;       % Boss centreline to ball centreline offset [mm]
    I_pyr_upper = 1864.34; % Second moment of area [mm^4]
    y_upper = 6.12;       % Distance from neutral axis to extreme fibre [mm]
    
    % ========== LOWER PYRAMID ADAPTER GEOMETRY ==========
    % Plate dimensions
    L_plate = 51.91;    % Plate side length [mm]
    t_plate = 3;        % Plate thickness [mm]
    
    % Bolt specifications
    d_bolt = 5.0;       % Bolt diameter [mm]
    n_bolts = 4;        % Number of bolts
    A_bolt = 14.2;      % Bolt tensile stress area [mm²]
    A_bearing = 15.00;  % Bearing area per bolt (d_bolt × t_plate) [mm²]
    r_bc = 25.48;       % Bolt circle radius (centre to bolt) [mm]
    F_bolt = 516.17;    % Shear force per bolt (F_res / n_bolts) [N]
    
    % Pyramid boss dimensions (identical to upper)
    b_pyr_lower = 12.23;  % Pyramid boss side length [mm]
    A_pyr_lower = 149.57; % Cross-sectional area [mm²]
    h_boss_lower = 9;     % Boss height [mm]
    e_lower = 0;          % Eccentricity (e = 0, no bending at boss) [mm]
    I_pyr_lower = 1864.34; % Second moment of area [mm^4]
    y_lower = 6.12;       % Distance to extreme fibre [mm]
    
end

methods (Static)

    % ========== COMPATIBILITY METHODS (for Main.m integration) ==========
    
    % getFoS_yield_u: Returns yield safety factor for upper adapter.
    % Inputs Fx and Fy are intentionally ignored here to preserve the
    % original Main.m call signature.
    function FoS_yield_u = getFoS_yield_u(~, ~, Sy)
        % sigma_vm: von Mises equivalent stress at upper adapter [MPa]
        [~, ~, sigma_vm, ~] = PyramidAdapter.analyzeUpperCombinedStress();
        % FoS_yield_u: yield safety factor = Sy / sigma_vm
        FoS_yield_u = Sy / sigma_vm;
    end
    
    % getFoS_yield_b: Returns yield safety factor for lower adapter.
    % Inputs Fx and Fy are intentionally ignored here to preserve the
    % original Main.m call signature.
    function FoS_yield_b = getFoS_yield_b(~, ~, Sy)
        % sigma_vm: von Mises equivalent stress at lower adapter [MPa]
        [~, ~, sigma_vm, ~] = PyramidAdapter.analyzeLowerCombinedStress();
        % FoS_yield_b: yield safety factor = Sy / sigma_vm
        FoS_yield_b = Sy / sigma_vm;
    end

    % ========== UPPER ADAPTER ANALYSIS ==========
    
    % Thread Stripping Analysis (Section 4.4.4)
    % Shear stress in engaged threads at the ball neck connection
    % tau_s = F_res / A_s_total
    function [tau_s, n_strip, pass] = analyzeThreadStripping()
        tau_s = PyramidAdapter.F_res / PyramidAdapter.A_s_total; % [MPa]
        tau_allow = PyramidAdapter.tau_y / PyramidAdapter.n_safety; % [MPa]
        n_strip = PyramidAdapter.tau_y / tau_s;
        pass = (tau_s <= tau_allow);
    end
    
    % Pyramid Boss Axial Compression (Section 4.4.5)
    % Vertical load causes direct axial stress on the hollow square pyramid boss
    function [sigma_axial, n_axial] = analyzeUpperAxialCompression()
        sigma_axial = PyramidAdapter.F_v / PyramidAdapter.A_pyr_upper; % [MPa]
        n_axial = (PyramidAdapter.sigma_y / PyramidAdapter.n_safety) / sigma_axial;
    end
    
    % Pyramid Boss Bending (Section 4.4.5)
    % Eccentricity e = 1.51 mm between pyramid boss centroid and ball centroid
    % creates bending moment M = F_v × e
    function [sigma_bending, n_bending] = analyzeUpperBending()
        M_upper = PyramidAdapter.F_v * PyramidAdapter.e_upper; % [N·mm]
        sigma_bending = (M_upper * PyramidAdapter.y_upper) / PyramidAdapter.I_pyr_upper; % [MPa]
        n_bending = (PyramidAdapter.sigma_y / PyramidAdapter.n_safety) / sigma_bending;
    end
    
    % Combined Stress at Upper Pyramid Boss (Section 4.4.6)
    % Von Mises criterion: sigma_e = sqrt(sigma_n^2 + 3*tau_AP^2)
    function [sigma_total, tau_AP, sigma_vm, n_vm] = analyzeUpperCombinedStress()
        % Normal stress from axial compression and bending
        sigma_axial = PyramidAdapter.F_v / PyramidAdapter.A_pyr_upper;
        M_upper = PyramidAdapter.F_v * PyramidAdapter.e_upper;
        sigma_bending = (M_upper * PyramidAdapter.y_upper) / PyramidAdapter.I_pyr_upper;
        sigma_total = sigma_axial + sigma_bending; % [MPa]
        
        % Shear stress from AP force (using factor 1.5 for rectangular cross-section)
        tau_AP = 1.5 * PyramidAdapter.F_AP / PyramidAdapter.A_pyr_upper; % [MPa]
        
        % Von Mises equivalent stress
        sigma_vm = sqrt(sigma_total^2 + 3 * tau_AP^2); % [MPa]
        n_vm = PyramidAdapter.sigma_y / sigma_vm;
    end
    
    % ========== LOWER ADAPTER ANALYSIS ==========
    
    % Bolt Shear Analysis (Section 4.5.4)
    % Resultant force shared equally among four bolts
    function [tau_bolt, n_bolt, pass] = analyzeBoltShear()
        tau_bolt = PyramidAdapter.F_bolt / PyramidAdapter.A_bolt; % [MPa]
        tau_allow = PyramidAdapter.tau_y / PyramidAdapter.n_safety; % [MPa]
        n_bolt = PyramidAdapter.tau_y / tau_bolt;
        pass = (tau_bolt <= tau_allow);
    end
    
    % Bolt Bearing Stress Analysis (Section 4.5.5)
    % Bearing area per bolt = d_bolt × t_plate
    % Allowable bearing stress = 1.5 × sigma_y
    function [sigma_bearing, n_bearing, pass] = analyzeBoltBearing()
        sigma_bearing = PyramidAdapter.F_bolt / PyramidAdapter.A_bearing; % [MPa]
        sigma_bearing_allow = 1.5 * PyramidAdapter.sigma_y; % [MPa]
        n_bearing = sigma_bearing_allow / sigma_bearing;
        pass = (sigma_bearing <= sigma_bearing_allow);
    end
    
    % Pyramid Boss Axial Compression (Lower Adapter, Section 4.5.6)
    % Same cross-section as upper adapter
    function [sigma_axial, n_axial] = analyzeLowerAxialCompression()
        sigma_axial = PyramidAdapter.F_v / PyramidAdapter.A_pyr_lower; % [MPa]
        n_axial = (PyramidAdapter.sigma_y / PyramidAdapter.n_safety) / sigma_axial;
    end
    
    % Combined Stress at Lower Pyramid Boss (Section 4.5.7)
    % No bending contribution (e = 0), only axial and AP shear
    % sigma_e = sqrt(sigma_axial^2 + 3*tau_AP^2)
    function [sigma_axial, tau_AP, sigma_vm, n_vm] = analyzeLowerCombinedStress()
        % Axial stress only (e = 0, no bending)
        sigma_axial = PyramidAdapter.F_v / PyramidAdapter.A_pyr_lower; % [MPa]
        
        % Shear stress from AP force
        tau_AP = 1.5 * PyramidAdapter.F_AP / PyramidAdapter.A_pyr_lower; % [MPa]
        
        % Von Mises equivalent stress
        sigma_vm = sqrt(sigma_axial^2 + 3 * tau_AP^2); % [MPa]
        n_vm = PyramidAdapter.sigma_y / sigma_vm;
    end
    
    % ========== SUMMARY RESULTS FUNCTION ==========
    
    % Generate complete analysis summary for both adapters
    function results = analyzeComplete()
        results = struct();
        
        % Upper Adapter Results
        [results.upper.tau_s, results.upper.n_strip, results.upper.thread_pass] = PyramidAdapter.analyzeThreadStripping();
        [results.upper.sigma_axial, results.upper.n_axial] = PyramidAdapter.analyzeUpperAxialCompression();
        [results.upper.sigma_bending, results.upper.n_bending] = PyramidAdapter.analyzeUpperBending();
        [results.upper.sigma_total, results.upper.tau_AP, results.upper.sigma_vm, results.upper.n_vm] = PyramidAdapter.analyzeUpperCombinedStress();
        
        % Lower Adapter Results
        [results.lower.tau_bolt, results.lower.n_bolt, results.lower.bolt_pass] = PyramidAdapter.analyzeBoltShear();
        [results.lower.sigma_bearing, results.lower.n_bearing, results.lower.bearing_pass] = PyramidAdapter.analyzeBoltBearing();
        [results.lower.sigma_axial, results.lower.n_axial] = PyramidAdapter.analyzeLowerAxialCompression();
        [results.lower.sigma_axial_vm, results.lower.tau_AP, results.lower.sigma_vm, results.lower.n_vm] = PyramidAdapter.analyzeLowerCombinedStress();
        
        % Material & Design Info
        results.material = 'AISI 316L Stainless Steel';
        results.sigma_y = PyramidAdapter.sigma_y;
        results.tau_y = PyramidAdapter.tau_y;
        results.F_v = PyramidAdapter.F_v;
        results.F_AP = PyramidAdapter.F_AP;
        results.n_safety = PyramidAdapter.n_safety;
    end

end
end

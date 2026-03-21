% =========================================================
% LATCH LOCK - ROTATING LATCH PARAMETRIZATION
% =========================================================

%{

% =========================================================
% INPUTS
% =========================================================
Material: AISI 304 Stainless Steel 

r_ball : radius of the ball [m]
M_k : maximum knee moment [Nm]
M_T : user inputted torque to engage lock [Nm]

% =========================================================
% LOCKING MECHANISM PARAMETERS
% =========================================================
Spring:
k : lock spring constant [kg/s^2]
y : deformation distance of the spring [m]
y_pre : preload compression [m]

Socket:
h_hump : height of the hump [m]

Pin:
d_pin : diameter of the pin [m]
l_pin : length of the pin [m]

Latch:
t_latch : thickness of the latch [m]

Handle:
l_handle : length of the handle

%}

classdef Lock

properties (Constant)

    % Material properties of the lock
    Sy = 276e6;   % [Pa], from Juvinall

    % Material properties of the spring
    G_spring = 69e9;   % shear modulus [Pa]
    C_index = 8;        % spring index
    N_a     = 6;        % active coils

    % Safety factor
    n = 4;  % TBC but greater than 2 ideally

    % Lock geometry (arbitraly fixed dimensions)
    h_hump = 2e-3; % [m]
    y_pre = 4e-3; % [m]
    t_latch = 10e-3;   % [m]
    l_handle = 40e-3;  % [m]

    % Engagement/disengagement torsion
    M_T = 0.378; % [Nm], from source

end

methods (Static)

% =========================================================
%  FORCES & MOMENTS
% =========================================================

    % calculate shear yield strength
    function Ssy = Ssy()
        Ssy = 0.577*Lock.Sy;
    end

    % calculate the user inputted force to engage/disengage the lock
    function F_T = F_T()
        F_T = Lock.M_T/Lock.l_handle;
    end
    
    % calculate the reaction moment required to keep the knee from bending
    % due to the weight of the lower leg and foot
    function M_leg = M_leg(m_f, m_LL, d_f, d_LL)
        M_leg = m_LL*d_LL + m_f*d_f;
    end

% =========================================================
% SPRING STIFFNESS
% =========================================================

    % calculate the spring stiffness required to acheive easy
    % engagement/disengagement
    function k = k()
        k = Lock.F_T()/(Lock.h_hump + Lock.y_pre);
    end

% =========================================================
%  SPRING GEOMETRY - INCOMPLETE BUT DOES NOT AFFECT ANYTHING ELSE
% =========================================================

    function N_total = N_total()
        N_total = Lock.N_a + 2;
    end

    % --- Wire diameter (correct equation) ---
    function d_w = d_w(k)
        d_w = ((8 * k * Lock.C_index^3 * Lock.N_a) / Lock.G_spring)^(1/4);
    end

    % --- Mean coil diameter ---
    function D_coil = D_coil(d_w)
        D_coil = Lock.C_index * d_w;
    end

    % --- Verify spring rate ---
    function k_verify = verifyK(d_w)
        k_verify = (Lock.G_spring * d_w^4) / (8 * Lock.D_coil(d_w)^3 * Lock.N_a);
    end

    % --- Forces ---
    function F_preload = F_preload(k)
        F_preload = k * Lock.delta_pre;
    end

    function F_max = F_max(k)
        F_max = k * (delta_pre + delta);
    end

    % --- Wahl correction factor ---
    function K_wahl = K_wahl()
        K_wahl = (4*Lock.C_index - 1)/(4*Lock.C_index - 4) + 0.615/Lock.C_index;
    end
    
    % --- Spring stress ---
    function tau_spring = tau_spring(k)
        tau_spring = Lock.K_wahl() * (8 * Lock.F_max(k) * Lock.D_coil) / (pi * Lock.d_w(k)^3);
    end
    
    % --- Safety factor ---
    function n_spring = n_spring(k)
        n_spring = Lock.Ssy / Lock.tau_spring(k);
    end
    
    % --- Solid length ---
    function l_solid = l_solid()
        l_solid = Lock.N_total() * Lock.d_w(k);
    end

%{
OLD DISPLAY FUNCTIONS
fprintf('=== SPRING GEOMETRY ===\n');
fprintf('Wire diameter: %.2f mm\n',d_w);
fprintf('Mean coil diameter: %.2f mm\n',D_coil);
fprintf('Spring rate verified: %.2f N/mm\n',k_verify);
fprintf('Solid length: %.2f mm\n',L_solid);
fprintf('Preload force: %.2f N\n',F_preload);
fprintf('Max spring force: %.2f N\n',F_max);
fprintf('Spring stress: %.2f MPa\n',tau_spring);
fprintf('Spring safety factor: %.2f\n\n',n_spring);
%}

% =========================================================
% KEYS
% =========================================================

    % calculate the compression force on the latch when engaged
    function F_C = F_C(M, r_ball)
        F_C = M/r_ball;
    end

    % solve for the width of the key required to withstand shear forces
    function w_key = w_key(F_C, k, y, tau_key)
        A_shear = (F_C - k*y)/(2*tau_key);
        w_key = A_shear/Lock.t_latch;
    end

% =========================================================
% PIN
% =========================================================

    % calculate the limiting pin diameter due to shear
    function d_pin = d_pin_shear(F_C, tau_pin)
        A_pin = F_C/tau_pin;
        d_pin = sqrt(4*A_pin/pi);
    end

    % calculate the limiting pin diameter due to torsion
    function d_pin = d_pin_torsion(tau_pin)
        d_pin = (16*Lock.M_T/(pi*tau_pin))^(1/3);
    end

    % calculate the limiting pin diameter due to bending
    function d_pin = d_pin_bending(F_C, sigma_pin)
        M_b = F_C*(0.5*Lock.t_latch);
        d_pin = (32*M_b/(pi*sigma_pin))^(1/3);
    end

% =========================================================
%  LATCH HERTZ CONTACT STRESS
% =========================================================

% TBD... we decided to maybe not to proceed with this analysis

% =========================================================
%  MAIN LOCK DIMENSIONS
% =========================================================

    function LockDim = LockDim(m_f, m_ll, d_f, d_ll, M_k, r_ball)

        LockDim.k = Lock.k();

        M = max(M_k,Lock.M_leg(m_f, m_ll, d_f, d_ll));
        F_C = Lock.F_C(M, r_ball);
        tau_allowable = Lock.Ssy()/Lock.n;
        sigma_allowable = Lock.Sy/Lock.n;

        LockDim.t_latch = Lock.t_latch;
        LockDim.w_key = Lock.w_key(F_C,LockDim.k,Lock.h_hump + Lock.y_pre,tau_allowable);

        d_pin_shear = Lock.d_pin_shear(F_C, tau_allowable);
        d_pin_torsion = Lock.d_pin_torsion(tau_allowable);
        d_pin_bending = Lock.d_pin_bending(F_C, sigma_allowable);
        LockDim.d_pin = max([d_pin_shear,d_pin_torsion,d_pin_bending]);
            
    end

end

end
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
    Su = 2200e6;    % [Pa], from Juvinall for ASTM A313 for 1<d<3 mm
    G_spring = 73e6;    % [Pa], from Juvinall stainless steel

    % Safety factor
    n = 4;

    % Lock geometry (arbitraly fixed dimensions)
    h_hump = 2e-3; % [m]
    y_pre = 4e-3; % [m]
    t_latch = 15e-3;   % [m]
    l_handle = 30e-3;  % [m]

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

    function F_pre = F_preload(k, preload_deflection)
        F_pre = k*preload_deflection;
    end

% =========================================================
% SPRING STIFFNESS
% =========================================================

    % calculate the maximum spring stiffness allowed to acheive easy
    % engagement/disengagement [N/m]
    function k = k(F_T)
        k = F_T/(Lock.h_hump + Lock.y_pre);
    end

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
        M_b = F_C*(0.75*Lock.t_latch);
        d_pin = (32*M_b/(pi*sigma_pin))^(1/3);
    end

% =========================================================
%  SPRING GEOMETRY
% =========================================================

function C = C(D,d)
    C = D/d;
end

function Ks = Ks(C)
    Ks = 1 + (0.5/C);
end

function Kw = Kw(C)
    Kw = (4*C-1)/(4*C-4) + (0.615/C);
end

% number of active coils
function N = N(d, k, D, G)
    N = ceil(G*d^4/(8*k*D^3));
end

function tau = tau(F, D, d, Kw)
    tau = 8*F*D*Kw/(pi*d^3);
end

% =========================================================
%  MAIN LOCK DIMENSIONS
% =========================================================

    function LockDim = LockDim(log, m_f, m_ll, d_f, d_ll, M_k, r_ball)

        fprintf(log, 'Set safety factor: %d\n\n', Lock.n);
        fprintf(log, 'Limiting lock dimensions:\n');
       
        % Limiting lock dimensions
        T = Lock.M_T;
        fprintf(log, 'Torque to engage/disengage lock: %.3f Nm\n', T);
        F_T = Lock.F_T();
        k = Lock.k(F_T);
        fprintf(log, 'Spring constant: %.2f N/m\n', k);

        M = max(M_k,Lock.M_leg(m_f, m_ll, d_f, d_ll));
        F_C = Lock.F_C(M, r_ball);
        fprintf(log, 'Maximum force on the latch: %.2f N\n', F_C);
        tau_allowable = Lock.Ssy()/Lock.n;
        sigma_allowable = Lock.Sy/Lock.n;
        fprintf(log, 'Maximum allowable shear stress: %.2f Pa\n', tau_allowable);
        fprintf(log, 'Maximum allowable bending stress: %.2f Pa\n', sigma_allowable);

        t_latch = Lock.t_latch;
        fprintf(log, 'Set latch thickness: %.2f mm\n', t_latch*1000);
        w_key = Lock.w_key(F_C,k,Lock.h_hump + Lock.y_pre,tau_allowable);
        fprintf(log, 'Minimum width of pin keys: %.2f mm\n', w_key*1000);

        d_pin_shear = Lock.d_pin_shear(F_C, tau_allowable);
        d_pin_torsion = Lock.d_pin_torsion(tau_allowable);
        d_pin_bending = Lock.d_pin_bending(F_C, sigma_allowable);
        d_pin = max([d_pin_shear,d_pin_torsion,d_pin_bending]);
        fprintf(log, 'Limiting pin diameter due to shear: %.2f mm\n', d_pin_shear*1000);
        fprintf(log, 'Limiting pin diameter due to torsion: %.2f mm\n', d_pin_torsion*1000);
        fprintf(log, 'Limiting pin diameter due to bending: %.2f mm\n', d_pin_bending*1000);
        fprintf(log, 'Minimum pin diameter: %.2f mm\n', d_pin*1000);
        
        % Limiting spring dimensions
        fprintf(log, '\nSpring dimensions:\n');
        D = d_pin*0.75;
        d = 0.001;  % to start, set d = 1mm
        while (true)
            C = Lock.C(D,d);
            fprintf(log, 'D (mean spring diameter): %.2f mm\n', D*1000);
            fprintf(log, 'd (wire diameter): %.2f mm\n', d*1000);
            fprintf(log, 'C:    %.2f\n', C);
   
            if (C >= 4 && C <= 12)
                fprintf(log, 'C within acceptable bounds of 4-12\n');
                break;
            elseif (C < 4)
                fprintf(log, 'C is outside of acceptable bound, C < 4\n');
                D = D + 0.001;  % increase D
            elseif (C > 12)
                fprintf(log, 'C is outside of acceptable bound, C > 12\n');
                d = d + 0.001;  % increase d
            end
        end

        Ks = Lock.Ks(C);
        Kw = Lock.Kw(C);
        N = Lock.N(d, k, D, Lock.G_spring);
        Nt = N + 2;
        Ls = Nt*d;
        fprintf(log, 'Ks: %.2f\n', Ks);
        fprintf(log, 'Kw: %.2f\n', Kw);
        fprintf(log, 'Number of active coils, N: %.0f\n', N);
        fprintf(log, 'Total number of coils, N_t: %.0f\n', Nt);
        fprintf(log, 'Solid length of the spring, L_s: %.2f mm\n', Ls*1000);

        F_min = Lock.F_preload(k, Lock.y_pre);
        F_max = Lock.F_T();

        tau_min = Lock.tau(F_min, D, d, Kw);
        tau_max = Lock.tau(F_max, D, d, Kw);

        tau_m = (tau_max + tau_min)/2;
        tau_a = (tau_max - tau_min)/2;

        % Static load yield
        fprintf(log, 'Static loading condition:\n');
        tau_s_allowed = 0.45*Lock.Su;
        tau_s = 8*F_max*C*Ks/(pi*d^2);
        static_condition_met = tau_s < tau_s_allowed;
        fprintf(log, 'Allowable static stress: %.2f Pa\n', tau_s_allowed);
        fprintf(log, 'Calculated static stress: %.2f Pa\n', tau_s);
        if (static_condition_met)
            fprintf(log, 'Static condition is met!\n');
        else
            fprintf(log, 'Static condition has not been met.\n');
        end

        fprintf(log, '\nSpring safety factors:\n');

        % Modified Goodman for spring
        Ssu = 0.8*Lock.Su;
        Ssn = 0.31*Lock.Su;
        n_goodman = 1/(tau_a/Ssn + tau_m/Ssu);
        fprintf(log, 'Goodman, n = %.2f\n', n_goodman);

        % Langer (oscillating load)
        Ssy = 0.53*Lock.Su;
        n_langer = 1/(tau_a/Ssy + tau_m/Ssy);
        fprintf(log, 'Langer, n = %.2f\n\n', n_langer);

        % Final lock dimensions
        fprintf(log, 'Final lock dimensions:\n');
        if (w_key < d_pin/4)
            LockDim.w_key = d_pin/4;
        else
            LockDim.w_key = w_key;
        end
        LockDim.d_pin = d_pin;
        LockDim.t_latch = t_latch;
        % free length of the spring
        LockDim.Lf = Ls + Lock.y_pre + Lock.h_hump + 0.002; % free length of the spring with 2mm buffer
        % latch is shaped like a slot (rectangle with two semi-circles on each end), l_latch is the length of the rectangle
        LockDim.l_latch = d_pin/2 + LockDim.Lf - Lock.y_pre + 0.010 - d_pin;
        LockDim.w_latch = d_pin*2;
        LockDim.k_spring = k;
        LockDim.D_spring = D;
        LockDim.d_spring = d;
        LockDim.Nt = Nt;
        LockDim.l_handle = Lock.l_handle;
        LockDim.w_handle = d_pin*2;

        fprintf(log, 'Width of pin keys, w_key: %.2f mm\n', LockDim.w_key*1000);
        fprintf(log, 'Pin diameter, d_pin: %.2f mm\n', LockDim.d_pin*1000);
        fprintf(log, 'Latch thickness, t_latch: %.2f mm\n', LockDim.t_latch*1000);
        fprintf(log, 'Free length of spring: %.2f mm\n', LockDim.Lf*1000);
        fprintf(log, 'Length of the latch, l_latch: %.2f mm\n', LockDim.l_latch*1000);
        fprintf(log, 'Width of the latch, w_latch: %.2f mm\n', LockDim.w_latch*1000);
        fprintf(log, 'Length of the handle, l_handle: %.2f mm\n', LockDim.l_handle*1000);
        fprintf(log, 'Width of the handle, w_handle: %.2f mm\n', LockDim.w_handle*1000);
        fprintf(log, 'Spring diameter, D_spring: %.2f mm\n', LockDim.D_spring*1000);
        fprintf(log, 'Coil diameter, d_spring: %.2f mm\n', LockDim.d_spring*1000);
        fprintf(log, 'Number of coils, Nt: %.0f mm\n\n', LockDim.Nt);

    end

end

end
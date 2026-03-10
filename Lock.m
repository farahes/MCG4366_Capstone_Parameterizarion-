% =========================================================
% LATCH LOCK - ROTATING LATCH PARAMETRIZATION
% MCG4366 Group 12
%
% Design logic:
% 1. Sweep theta (hump angle) and compute feasible k window
% 2. k_min comes from back-drive requirement
% 3. k_max comes from user unlock force limit
% 4. Spring geometry is back-calculated from chosen k and delta
% 5. Pin shear and Hertzian contact checked at each theta
% =========================================================

%{

% =========================================================
% INPUTS
% =========================================================
Material: AISI 304 Stainless Steel

r_ball : radius of the ball [m]
M_k : maximum knee moment [Nm]
F_lock : user inputted force to engage lock [N]

% =========================================================
% LOCKING MECHANISM PARAMETERS
% =========================================================
Spring:
k : lock spring constant [kg/s^2]
delta : deformation distance of the spring (~= h_hump) [m]
delta_pre : preload compression [m]

Socket:
h_hump : height of the hump [m]

Pin:
d_pin : diameter of the pin [m]

Latch:
r_latch : radius of the latch tip [m]
l_cavity : length of the spring cavity [m]

Handle:
l_handle : length of the handle

%}

classdef Lock

properties (Constant)

    % Material properties of the lock
    Sy = 207e6;   % [Pa]
    E = 193e9;  % [Pa]
    nu = 0.29;  % Poisson's ratio

    % Material properties of the spring
    G_spring = 69e9;   % shear modulus [Pa]
    C_index = 8;        % spring index
    N_a     = 6;        % active coils

    % Safety factor
    n = 2;

    % Lock geometry (FIXED DIMENSIONS)
    h_hump = 2e-3; % [m]
    l_cavity = 20e-3; % [m]
    delta_pre = 4e-3; % preload compression [m]
    d_pin = 8e-3;  % [m]
    r_latch = 5e-3;    % radius of the latch tip [m]
    t_latch = 10e-3;   % thickness of the latch [m]
    l_handle = 30e-3;  % [m]
    F_lock = 50;    % [N] where does this come from???

end

methods (Static)

% =========================================================
%  FORCES & MOMENTS
% =========================================================

    % calculate shear yield strength
    function Ssy = Ssy()
        Ssy = 0.577*Lock.Sy;
    end

    % return the maximum moment in Nm
    function M = maxM(M1, M2)
        if M1 >= M2
            M = M1;
        else
            M = M2;
        end
    end
    
    % calculate the reaction moment required to keep the knee from bending
    % due to the weight of the lower leg and foot
    function M = reactM(m_f, m_LL, d_f, d_LL)
        M = m_LL*d_LL + m_f*d_f;
    end

    % calculate the compression force on the latch from the knee moment
    function F = FLatch(M, r_b)
        F = M/r_b;
    end

% =========================================================
% SPRING LENGTH
% =========================================================

    function l_free = getLfree()
        l_free = Lock.l_cavity + Lock.delta_pre;
    end
    
    % operating length
    function l_op = getLop()
        l_op = Lock.l_cavity - Lock.delta();
    end

% =========================================================
% LATCH ANGLE & SPRING STIFFNESS
% =========================================================

    function delta_ = delta()
        delta_ = Lock.h_hump;
    end

    % i don't understand this calculation
    % what is theta?
    % and how are k_min and k_max calculated?
    function k = getK(F_latch, r_ball)
        theta_deg = linspace(10,80,500);
        theta_rad = deg2rad(theta_deg);
        
        % --- Minimum stiffness needed to resist knee load ---
        k_min = (F_latch*tan(theta_rad))/(r_ball*Lock.delta_pre);
        
        % --- Maximum stiffness user can overcome ---
        k_max = (Lock.F_lock*Lock.l_handle)/((Lock.delta_pre + Lock.delta())*r_ball);
        
        feasible = k_min <= k_max;
        
        %fprintf('=== STIFFNESS BOUNDS ===\n');
        %fprintf('User stiffness limit: %.2f N/mm\n',k_max);
        
        if any(feasible)
        
            theta_feas = theta_deg(feasible);
            theta_lo   = min(theta_feas);
            theta_hi   = max(theta_feas);
            theta_rec  = mean([theta_lo theta_hi]);
        
            k_required = (F_latch*tand(theta_rec))/(r_ball*Lock.delta_pre);
        
            k = mean([k_required k_max]);
        
            %fprintf('Feasible theta: %.1f° to %.1f°\n',theta_lo,theta_hi);
            %fprintf('Recommended theta: %.1f°\n',theta_rec);
            %fprintf('Chosen spring stiffness: %.2f N/mm\n\n',k);
        
        else
        
            %fprintf('NO FEASIBLE ANGLE\n\n');
            k = k_max;
        
        end
    end

% =========================================================
%  SPRING GEOMETRY
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
% PIN SHEAR CHECK
% =========================================================

% YOU ALSO NEED TORSION ON THE PIN
% AND SHEAR OF THE KEYS ON THE PIN

    % shear area of the pin
    function A_pin = A_pin()
        A_pin = (pi*Lock.d_pin^2)/4;
    end

    function tau_pin = tau_pin(F_latch)
        tau_pin = F_latch / Lock.A_pin();
    end

    function n_pin = n_pin(F_latch)
        n_pin = Lock.Ssy / Lock.tau_pin(F_latch);
    end

%fprintf('=== PIVOT BOLT CHECK ===\n');
%fprintf('Bolt shear stress: %.2f MPa\n',tau_bolt);
%fprintf('Safety factor: %.2f\n\n',n_bolt);

% =========================================================
%  LATCH HERTZ CONTACT STRESS
% =========================================================

function r_socket = r_socket()
    r_socket = Lock.r_latch + 0.5;   % socket radius [mm]
end

function n_hertz = n_hertz(k)

    %{
    % Effective modulus
    E_star = Lock.E / (2*(1-Lock.nu^2));
    
    % Effective radius
    R_eff = (R_contact * R_ball) / (R_contact + R_ball);    % you can't use r_ball here...that's the completely wrong diameter, also this is for convex-convex...we want concave-convex

    % Contact radius (Hertz equation) Where did this version of the Hertz
    % equation come from?
    a_contact = ((3 * F_preload * R_eff) / (4 * E_star))^(1/3);

    % Maximum pressure
    p_max = (3 * Lock.F_preload(k)) / (2*pi*a_contact^2);
    %}

    % Curved surface contact stress equation comes from Juvinall and
    % Marshek, chapter 9.13 equations

    % Contact modulus
    delta_hertz = (1 - Lock.nu^2)/Lock.E + (1 - nu_ball^2)/E_ball;  % NEED TO GET THE BALL MATERIAL PROPERTIES TO PUT HERE

    % Maximum pressure
    p_max = 0.564*sqrt(Lock.preload(k)*((1/Lock.r_latch) + (1/Lock.r_socket()))/(Lock.t_latch*delta_hertz));

    % Safety factor
    n_hertz = Lock.Sy / p_max;
end

%fprintf('=== HERTZ CONTACT ===\n');
%fprintf('Contact radius: %.4f mm\n',a_contact);
%fprintf('Peak pressure: %.2f MPa\n',p_max);
%fprintf('Safety factor: %.2f\n\n',n_hertz);

% =========================================================
%  MAIN LOCK DIMENSIONS
% =========================================================

    function LockDim = LockDim(m_f, m_LL, d_f, d_LL, M_k, r_ball)

        % --- Get the maximum moment the lock needs to sustain ---
        M_react = Lock.reactM(m_f, m_LL, d_f, d_LL);
        M_max = Lock.maxM(M_k, M_react);

        % --- Calculate the compression force on the latch ---
        F_latch = Lock.FLatch(M_max, r_ball);

        % check safety factors, iterate if necessary
        % return all lock dimensions once safety factor is good

        k = Lock.getK(F_latch, r_ball);
            
    end

end

end

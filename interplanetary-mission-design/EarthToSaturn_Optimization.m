%% =========================================================================
%  Earth to Saturn — Constrained Trajectory Optimization
%  Author: D. J, Bryan L., S.V., S. M.
%
%  Problem: Minimize total delta-V for a non-Hohmann elliptical transfer
%  from an Earth parking orbit to a Saturn capture orbit.
%
%  Design Variables (5):
%    x1 — Periapsis altitude above Earth    (km)
%    x2 — Apoapsis altitude above Earth     (km)
%    x3 — Periapsis altitude above Saturn   (km)
%    x4 — Apoapsis altitude above Saturn    (km)
%    x5 — True anomaly at Saturn encounter  (deg)
%
%  Outputs: Optimal delta-V budget, trajectory eccentricities, aiming
%           radii, turning angles, propellant fraction, time-of-flight.
% =========================================================================

clear; clc; close all;

%% --- Unit Conversions ---
sec_to_month = 3.802648620817372e-07;

%% --- Gravitational Parameters (km^3/s^2) ---
mu_earth  = 3.986012e5;
mu_saturn = 37931187;
mu_sun    = 132712440018;

%% --- Planetary Orbital Radii — Semimajor Axes (km) ---
R_earth   = 1.4959965e8;
R_saturn  = 1.433e9;

%% --- Planetary Body Radii (km) ---
r_earth   = 6378;
r_saturn  = 60270;

%% --- Propulsion Parameters ---
I_sp = 290;           % Specific impulse (sec)
g_o  = 9.81 / 1000;  % Standard gravity (km/s^2)

%% =========================================================================
%  OPTIMIZATION — Minimize Total Delta-V
% =========================================================================

prob = optimproblem;

% Design variables with physical bounds
x1 = optimvar('x1', 'LowerBound', 1000);                        % Earth periapsis alt (km)
x2 = optimvar('x2', 'LowerBound', 1000,  'UpperBound', 3600);   % Earth apoapsis alt (km)
x3 = optimvar('x3', 'LowerBound', 2.83e5);                      % Saturn periapsis alt (km)
x4 = optimvar('x4', 'LowerBound', 2.83e5, 'UpperBound', 54.8e6);% Saturn apoapsis alt (km)
x5 = optimvar('x5', 'LowerBound', 0,      'UpperBound', 180);   % True anomaly (deg)

% Objective function
obj        = fcn2optimexpr(@(x1,x2,x3,x4,x5) objfun(x1,x2,x3,x4,x5, ...
                mu_earth, mu_saturn, mu_sun, R_earth, R_saturn, r_earth, r_saturn), ...
                x1, x2, x3, x4, x5);
prob.Objective = obj;

% Initial guess
x0.x1 = 1000;
x0.x2 = 3600;
x0.x3 = 2.83e5;
x0.x4 = 54.8e6;
x0.x5 = 155;

[sol, fval, exitflag] = solve(prob, x0);

%% =========================================================================
%  POST-OPTIMIZATION — Reconstruct Full Trajectory from Optimal Solution
% =========================================================================

% Extract optimal design variables
z_p_earth  = sol.x1;
z_a_earth  = sol.x2;
z_p_saturn = sol.x3;
z_a_saturn = sol.x4;
theta      = sol.x5;

% Convert altitudes to radii
r_p_earth  = z_p_earth  + r_earth;
r_a_earth  = z_a_earth  + r_earth;
r_p_saturn = z_p_saturn + r_saturn;
r_a_saturn = z_a_saturn + r_saturn;

%% --- Heliocentric Transfer Ellipse ---
e_t = (R_saturn - R_earth) / (R_earth - R_saturn * cosd(theta));
h_t = sqrt(R_earth * mu_sun * (1 + e_t));

%% --- Departure from Earth ---
v_n_t_d    = (mu_sun / h_t) * (1 + e_t * cosd(0));
v_r_t_d    = (mu_sun / h_t) * e_t * sind(0);
v_earth    = sqrt(mu_sun / R_earth);
v_h_d      = norm([v_n_t_d - v_earth, v_r_t_d]);

e_h_d      = 1 + (r_p_earth * v_h_d^2) / mu_earth;
v_p_d      = sqrt(v_h_d^2 + (2 * mu_earth / r_p_earth));

e_p_earth  = (r_a_earth - r_p_earth) / (r_a_earth + r_p_earth);
a_p_earth  = (r_a_earth + r_p_earth) / 2;
p_p_earth  = a_p_earth * (1 - e_p_earth^2);
v_c_p_e    = sqrt(mu_earth / p_p_earth) * (1 + e_p_earth);

dv_depart  = v_p_d - v_c_p_e;

%% --- Arrival at Saturn ---
v_n_t_a    = (mu_sun / h_t) * (1 + e_t * cosd(theta));
v_r_t_a    = (mu_sun / h_t) * e_t * sind(theta);
v_saturn   = sqrt(mu_sun / R_saturn);
v_h_a      = norm([v_n_t_a - v_saturn, v_r_t_a]);

e_h_a      = 1 + (r_p_saturn * v_h_a^2) / mu_saturn;
e_p_saturn = (r_a_saturn - r_p_saturn) / (r_a_saturn + r_p_saturn);
v_p_capture = sqrt((mu_saturn * (1 + e_p_saturn)) / r_p_saturn);
v_p_arrive  = sqrt(v_h_a^2 + (2 * mu_saturn / r_p_saturn));

dv_arrive  = v_p_arrive - v_p_capture;

%% --- Delta-V Budget & Propellant ---
dv_total      = dv_depart + dv_arrive;
prop_fraction = (1 - exp(-dv_total / (I_sp * g_o))) * 100;

%% --- Hyperbolic Geometry ---
aiming_depart = r_p_earth  * sqrt((1 + e_h_d) / (e_h_d - 1));
turn_depart   = 2 * asind(1 / e_h_d);

aiming_arrive = r_p_saturn * sqrt((1 + e_h_a) / (e_h_a - 1));
turn_arrive   = 2 * asind(1 / e_h_a);

%% --- Time of Flight ---
T         = ((2 * pi) / mu_sun^2) * (h_t / sqrt(1 - e_t^2))^3;
theta_rad = theta * pi / 180;
E         = 2 * atan(sqrt((1 - e_t) / (1 + e_t)) * tan(theta_rad / 2));
Me        = E - e_t * sin(E);
tof_sec   = (Me / (2 * pi)) * T;
tof_month = floor(tof_sec * sec_to_month);
tof_day   = (tof_sec * sec_to_month - tof_month) * 30;

%% =========================================================================
%  OUTPUT
% =========================================================================

fprintf('\n');
fprintf('=================================================================\n');
fprintf('  Earth to Saturn — Optimal Trajectory Design Results\n');
fprintf('=================================================================\n\n');

fprintf('  OPTIMIZER STATUS\n');
fprintf('  -----------------\n');
fprintf('  Exit flag                   : %d\n', exitflag);
fprintf('\n');

fprintf('  OPTIMAL DESIGN VARIABLES\n');
fprintf('  --------------------------\n');
fprintf('  Earth periapsis altitude    : %+0.15e km\n', z_p_earth);
fprintf('  Earth apoapsis altitude     : %+0.15e km\n', z_a_earth);
fprintf('  Saturn periapsis altitude   : %+0.15e km\n', z_p_saturn);
fprintf('  Saturn apoapsis altitude    : %+0.15e km\n', z_a_saturn);
fprintf('  True anomaly at encounter   : %+0.15e deg\n', theta);
fprintf('\n');

fprintf('  DELTA-V BUDGET\n');
fprintf('  ---------------\n');
fprintf('  Departure (Earth escape)    : %+0.15e km/s\n', dv_depart);
fprintf('  Arrival   (Saturn capture)  : %+0.15e km/s\n', dv_arrive);
fprintf('  Total Delta-V               : %+0.15e km/s\n', dv_total);
fprintf('\n');

fprintf('  TRAJECTORY ECCENTRICITIES\n');
fprintf('  --------------------------\n');
fprintf('  Transfer ellipse            : %+0.15e ---\n', e_t);
fprintf('  Departure hyperbola (Earth) : %+0.15e ---\n', e_h_d);
fprintf('  Arrival hyperbola (Saturn)  : %+0.15e ---\n', e_h_a);
fprintf('\n');

fprintf('  HYPERBOLIC TRAJECTORY GEOMETRY\n');
fprintf('  --------------------------------\n');
fprintf('  Aiming radius  — departure  : %+0.15e km\n',  aiming_depart);
fprintf('  Aiming radius  — arrival    : %+0.15e km\n',  aiming_arrive);
fprintf('  Turning angle  — departure  : %+0.15e deg\n', turn_depart);
fprintf('  Turning angle  — arrival    : %+0.15e deg\n', turn_arrive);
fprintf('\n');

fprintf('  PROPELLANT & TIME OF FLIGHT\n');
fprintf('  ----------------------------\n');
fprintf('  Total propellant fraction   : %+0.15e %%\n',  prop_fraction);
fprintf('  Time of flight              : %d months, %.1f days\n', tof_month, tof_day);
fprintf('\n');
fprintf('=================================================================\n');

%% =========================================================================
%  OBJECTIVE FUNCTION
% =========================================================================

function f = objfun(x1, x2, x3, x4, x5, mu_E, mu_Sat, mu_Sun, R_E, R_Sat, r_E, r_Sat)
    % Computes total delta-V as a function of the 5 design variables.
    % All parameters passed explicitly — no workspace dependencies.

    % Radii from altitudes
    r_p_e = x1 + r_E;
    r_a_e = x2 + r_E;
    r_p_s = x3 + r_Sat;
    r_a_s = x4 + r_Sat;

    % Transfer ellipse
    e_t = (R_Sat - R_E) / (R_E - R_Sat * cosd(x5));
    h_t = sqrt(R_E * mu_Sun * (1 + e_t));

    % Departure
    v_n_d   = (mu_Sun / h_t) * (1 + e_t * cosd(0));
    v_r_d   = (mu_Sun / h_t) * e_t * sind(0);
    v_earth = sqrt(mu_Sun / R_E);
    v_h_d   = norm([v_n_d - v_earth, v_r_d]);

    e_h_d   = 1 + (r_p_e * v_h_d^2) / mu_E;
    v_p_d   = sqrt(v_h_d^2 + 2 * mu_E / r_p_e);

    e_p_e   = (r_a_e - r_p_e) / (r_a_e + r_p_e);
    a_p_e   = (r_a_e + r_p_e) / 2;
    p_p_e   = a_p_e * (1 - e_p_e^2);
    v_c_e   = sqrt(mu_E / p_p_e) * (1 + e_p_e);

    dv_d    = v_p_d - v_c_e;

    % Arrival
    v_n_a    = (mu_Sun / h_t) * (1 + e_t * cosd(x5));
    v_r_a    = (mu_Sun / h_t) * e_t * sind(x5);
    v_sat    = sqrt(mu_Sun / R_Sat);
    v_h_a    = norm([v_n_a - v_sat, v_r_a]);

    e_h_a    = 1 + (r_p_s * v_h_a^2) / mu_Sat;
    e_p_s    = (r_a_s - r_p_s) / (r_a_s + r_p_s);
    v_p_a    = sqrt(v_h_a^2 + 2 * mu_Sat / r_p_s);
    v_cap    = sqrt((mu_Sat * (1 + e_p_s)) / r_p_s);

    dv_a     = v_p_a - v_cap;

    f = dv_d + dv_a;
end
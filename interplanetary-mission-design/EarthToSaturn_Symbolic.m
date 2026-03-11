%% =========================================================================
%  Earth to Saturn — Symbolic Derivation of Total Delta-V
%  Authors: D. J, Bryan L., S.V., S. M.
%
%  Purpose: Symbolic derivation of the total delta-V cost function with
%  respect to the 5 design variables. Used to derive the analytical
%  expression passed to the numerical optimizer in EarthToSaturn_Optimization.m
%
%  Design Variables:
%    x1 — Periapsis altitude above Earth    (km)
%    x2 — Apoapsis altitude above Earth     (km)
%    x3 — Periapsis altitude above Saturn   (km)
%    x4 — Apoapsis altitude above Saturn    (km)
%    x5 — True anomaly at Saturn encounter  (deg)
% =========================================================================

clear; clc; close all;
format shortEng

%% --- Symbolic Variables ---
syms x1 x2 x3 x4 x5
syms mu_E mu_Sat mu_Sun R_E R_Sat r_E r_Sat I_sp g0 positive

%% --- Symbolic Parameter Assignments ---
mu_earth  = mu_E;
mu_saturn = mu_Sat;
mu_sun    = mu_Sun;
R_earth   = R_E;
R_saturn  = R_Sat;
r_earth   = r_E;
r_saturn  = r_Sat;
g_o       = g0 / 1000;

%% --- Design Variable Mapping ---
z_p_earth  = x1;
z_a_earth  = x2;
z_p_saturn = x3;
z_a_saturn = x4;
theta      = x5;

%% --- Altitude to Radius ---
r_p_earth  = z_p_earth  + r_earth;
r_a_earth  = z_a_earth  + r_earth;
r_p_saturn = z_p_saturn + r_saturn;
r_a_saturn = z_a_saturn + r_saturn;

%% --- Heliocentric Transfer Ellipse ---
e_t = (R_saturn - R_earth) / (R_earth - R_saturn * cosd(theta));
h_t = sqrt(R_earth * mu_sun * (1 + e_t));

%% --- Departure from Earth ---
v_n_t_d = (mu_sun / h_t) * (1 + e_t * cosd(0));
v_r_t_d = (mu_sun / h_t) * e_t * sind(0);
v_earth  = sqrt(mu_sun / R_earth);

v_h_d   = sqrt((v_n_t_d - v_earth)^2 + v_r_t_d^2);
e_h_d   = 1 + (r_p_earth * v_h_d^2) / mu_earth;
v_p_d   = sqrt(v_h_d^2 + (2 * mu_earth / r_p_earth));

e_p_e   = (r_a_earth - r_p_earth) / (r_a_earth + r_p_earth);
a_p_e   = (r_a_earth + r_p_earth) / 2;
p_p_e   = a_p_e * (1 - e_p_e^2);
v_c_e   = sqrt(mu_earth / p_p_e) * (1 + e_p_e);

dv_depart = v_p_d - v_c_e;

%% --- Arrival at Saturn ---
v_n_t_a  = (mu_sun / h_t) * (1 + e_t * cosd(theta));
v_r_t_a  = (mu_sun / h_t) * e_t * sind(theta);
v_saturn = sqrt(mu_sun / R_saturn);

v_h_a    = sqrt((v_n_t_a - v_saturn)^2 + v_r_t_a^2);
e_h_a    = 1 + (r_p_saturn * v_h_a^2) / mu_saturn;
e_p_s    = (r_a_saturn - r_p_saturn) / (r_a_saturn + r_p_saturn);
v_p_a    = sqrt(v_h_a^2 + (2 * mu_saturn / r_p_saturn));
v_cap    = sqrt((mu_saturn * (1 + e_p_s)) / r_p_saturn);

dv_arrive = v_p_a - v_cap;

%% --- Symbolic Total Delta-V ---
dv_total = dv_depart + dv_arrive;

fprintf('Symbolic total delta-V expression derived.\n');
fprintf('Simplifying — this may take a moment...\n\n');

dv_simplified = simplify(dv_total, 'Steps', 50);

fprintf('Total Delta-V (symbolic):\n');
disp(dv_simplified)

%% --- Optional: Display Partial Derivatives ---
% Useful for verifying gradient structure fed to the optimizer
fprintf('\nPartial derivatives with respect to design variables:\n');
fprintf('  d(dv)/d(x1) — Earth periapsis altitude:\n');
disp(simplify(diff(dv_total, x1), 'Steps', 20))

fprintf('  d(dv)/d(x5) — True anomaly:\n');
disp(simplify(diff(dv_total, x5), 'Steps', 20))
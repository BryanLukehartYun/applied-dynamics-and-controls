%% ========================================================================
%  6-DOF Nonlinear Fixed-Wing Flight Simulator — Main Script
%  ========================================================================
%  Author  : Bryan L, in conjunction with P.S and M.C
%             (co-authors anonymized at their discretion)
%  Course  : Flight Dynamics — MECE 410 (Predecessor to Graduate Version)
%  Date    : Circa Mid-2021
%  Purpose : Executes four linearized perturbation conditions against the
%            nonlinear truth model and produces a consolidated 2x2
%            diagnostic summary figure.
%
%  State Vector : [VT, alpha, q, beta, p, r, phi, theta, psi]^T
%  Control Vec  : [elevator (e), aileron (a), rudder (r)]^T
%
%  External Dependencies (treated as black-boxes):
%    - FixedWingSim_Parameters.m      : Aircraft/trim parameters (client input)
%    - FixedWingSim_SimulinkDiagram   : 6-DOF Simulink truth model
% =========================================================================

%% --- Workspace Initialization -------------------------------------------
clc;
clear;
run('FixedWingSim_Parameters.m');   % Load aircraft & trim parameters

%% --- Inertia Matrix Inverse (K) -----------------------------------------
% Used by the Simulink diagram to propagate angular acceleration.
% K = I^{-1}, where I is the full 3x3 inertia tensor.
K = inv([ Ixx, -Ixy, -Ixz;
         -Ixy,  Iyy, -Iyz;
         -Ixz, -Iyz,  Izz]);

%% ========================================================================
%  CONDITION 1 — TRIM VERIFICATION (Steady-State Equilibrium)
%  ========================================================================
%  Objective : Confirm the aircraft holds its trim state with zero control
%              perturbations over the full 100-second run.
%  Trim point: Vt = 626.818 ft/s | alpha = theta = 3.6103 deg
%              de = -3.0380 deg  | df = 1.5 deg | Thrust = 3146.48 lbs
% -------------------------------------------------------------------------
e = de0;   a = da0;   r = dr0;
sim_trim = sim('FixedWingSim_SimulinkDiagram.slx');

%% ========================================================================
%  CONDITION 2 — LONGITUDINAL STEP RESPONSE (Elevator Perturbation)
%  ========================================================================
%  Objective : Excite longitudinal modes (Short Period & Phugoid).
%  Input     : delta_e = -0.5 deg step at t = 1.0s => total = -3.5380 deg
%  Signature : Pitch rate (q) transient; alpha and theta growth; Vt decay.
% -------------------------------------------------------------------------
e = de0 - (0.5 * d2r);   a = da0;   r = dr0;
sim_elev = sim('FixedWingSim_SimulinkDiagram.slx');

%% ========================================================================
%  CONDITION 3 — LATERAL STEP RESPONSE (Aileron Perturbation)
%  ========================================================================
%  Objective : Excite roll mode and observe lateral-directional coupling.
%  Input     : delta_a = -0.5 deg step at t = 1.0s
%  Signature : Roll rate (p) buildup; bank angle (phi); sideslip (beta) and
%              yaw rate (r) coupling via Dihedral Effect.
% -------------------------------------------------------------------------
e = de0;   a = da0 - (0.5 * d2r);   r = dr0;
sim_ail = sim('FixedWingSim_SimulinkDiagram.slx');

%% ========================================================================
%  CONDITION 4 — DIRECTIONAL STEP RESPONSE (Rudder Perturbation)
%  ========================================================================
%  Objective : Excite Dutch Roll mode; assess directional stability.
%  Input     : delta_r = -2.0 deg step at t = 1.0s
%  Signature : Sideslip (beta) and yaw rate (r) oscillation; rolling moment
%              due to sideslip (Dihedral Effect) drives bank angle (phi).
% -------------------------------------------------------------------------
e = de0;   a = da0;   r = dr0 - (2.0 * d2r);
sim_rud = sim('FixedWingSim_SimulinkDiagram.slx');

%% ========================================================================
%  CONSOLIDATED 2x2 SUMMARY FIGURE
%  ========================================================================
%  Layout  : Each quadrant = one condition.
%            Axes within each quadrant highlight the state variables most
%            diagnostic of that condition's dominant flight mode.
%
%  Quad 1 (top-left)     — Trim Verification   : Vt, alpha, theta  (should be flat)
%  Quad 2 (top-right)    — Elevator Perturbation: q, alpha, Vt      (longitudinal modes)
%  Quad 3 (bottom-left)  — Aileron Perturbation : p, phi, beta      (roll + coupling)
%  Quad 4 (bottom-right) — Rudder Perturbation  : r, beta, phi      (Dutch Roll)
% -------------------------------------------------------------------------

figure('Name', '6-DOF Fixed-Wing: Condition Summary', ...
       'NumberTitle', 'off', ...
       'Units', 'normalized', ...
       'Position', [0.05, 0.05, 0.90, 0.85]);

sgtitle('6-DOF Fixed-Wing Simulator — Condition Response Summary', ...
        'FontSize', 14, 'FontWeight', 'bold');

%  Color palette — one distinct color per state variable for consistency
%  across quadrants (readers can cross-reference by color).
c_Vt    = [0.00, 0.45, 0.74];   % blue
c_alpha = [0.85, 0.33, 0.10];   % red-orange
c_theta = [0.93, 0.69, 0.13];   % gold
c_q     = [0.49, 0.18, 0.56];   % purple
c_p     = [0.47, 0.67, 0.19];   % green
c_r     = [0.30, 0.75, 0.93];   % cyan
c_beta  = [0.64, 0.08, 0.18];   % dark red
c_phi   = [1.00, 0.60, 0.00];   % orange

% ---- QUADRANT 1: Trim Verification (top-left) ---------------------------
ax1 = subplot(2, 2, 1);
hold(ax1, 'on');
plot(sim_trim.time, sim_trim.VT,    'Color', c_Vt,    'LineWidth', 1.5, 'DisplayName', 'V_T (ft/s)');
plot(sim_trim.time, sim_trim.alpha, 'Color', c_alpha, 'LineWidth', 1.5, 'DisplayName', '\alpha (deg)');
plot(sim_trim.time, sim_trim.theta, 'Color', c_theta, 'LineWidth', 1.5, 'DisplayName', '\theta (deg)');
title('Condition 1 — Trim Verification', 'FontWeight', 'bold');
xlabel('Time (sec)');  ylabel('State Value');
legend('show', 'Location', 'best', 'FontSize', 8);
grid on;  box on;

% ---- QUADRANT 2: Elevator Perturbation (top-right) ----------------------
ax2 = subplot(2, 2, 2);
hold(ax2, 'on');
plot(sim_elev.time, sim_elev.q,     'Color', c_q,     'LineWidth', 1.5, 'DisplayName', 'q (deg/s)');
plot(sim_elev.time, sim_elev.alpha, 'Color', c_alpha, 'LineWidth', 1.5, 'DisplayName', '\alpha (deg)');
plot(sim_elev.time, sim_elev.VT,    'Color', c_Vt,    'LineWidth', 1.5, 'DisplayName', 'V_T (ft/s)');
title('Condition 2 — Elevator (\Deltae = -0.5°)', 'FontWeight', 'bold');
xlabel('Time (sec)');  ylabel('State Value');
legend('show', 'Location', 'best', 'FontSize', 8);
grid on;  box on;

% ---- QUADRANT 3: Aileron Perturbation (bottom-left) ---------------------
ax3 = subplot(2, 2, 3);
hold(ax3, 'on');
plot(sim_ail.time, sim_ail.p,   'Color', c_p,    'LineWidth', 1.5, 'DisplayName', 'p (deg/s)');
plot(sim_ail.time, sim_ail.phi, 'Color', c_phi,  'LineWidth', 1.5, 'DisplayName', '\phi (deg)');
plot(sim_ail.time, sim_ail.beta,'Color', c_beta, 'LineWidth', 1.5, 'DisplayName', '\beta (deg)');
title('Condition 3 — Aileron (\Deltaa = -0.5°)', 'FontWeight', 'bold');
xlabel('Time (sec)');  ylabel('State Value');
legend('show', 'Location', 'best', 'FontSize', 8);
grid on;  box on;

% ---- QUADRANT 4: Rudder Perturbation (bottom-right) ---------------------
ax4 = subplot(2, 2, 4);
hold(ax4, 'on');
plot(sim_rud.time, sim_rud.r,    'Color', c_r,    'LineWidth', 1.5, 'DisplayName', 'r (deg/s)');
plot(sim_rud.time, sim_rud.beta, 'Color', c_beta, 'LineWidth', 1.5, 'DisplayName', '\beta (deg)');
plot(sim_rud.time, sim_rud.phi,  'Color', c_phi,  'LineWidth', 1.5, 'DisplayName', '\phi (deg)');
title('Condition 4 — Rudder (\Deltar = -2.0°)', 'FontWeight', 'bold');
xlabel('Time (sec)');  ylabel('State Value');
legend('show', 'Location', 'best', 'FontSize', 8);
grid on;  box on;


%% ========================================================================
%  LOCAL HELPER — plot_condition_debug(sim_out, condition_label)
%  ========================================================================
%  Full 3x3 diagnostic plot of all 9 state variables for a single condition.
%  Intended for command-window use when the 2x2 summary needs deeper
%  inspection on a specific run.
%
%  USAGE (from command window, AFTER the main script has run): UNCOMMENT
%  THE CHOICES BELOW.
%
%    plot_condition_debug(sim_trim, 'Trim Verification')
%    plot_condition_debug(sim_elev, 'Elevator Perturbation')
%    plot_condition_debug(sim_ail,  'Aileron Perturbation')
%    plot_condition_debug(sim_rud,  'Rudder Perturbation')
%
%  INPUTS:
%    sim_out         — Simulink output struct from any of the four sim() calls
%    condition_label — String used as the figure title
% -------------------------------------------------------------------------
function plot_condition_debug(sim_out, condition_label)
    % State definitions: { struct field name , y-axis label }
    states = { ...
        'VT',    'V_T (ft/s)';   ...
        'alpha', '\alpha (deg)'; ...
        'beta',  '\beta (deg)';  ...
        'p',     'p (deg/s)';    ...
        'q',     'q (deg/s)';    ...
        'r',     'r (deg/s)';    ...
        'phi',   '\phi (deg)';   ...
        'theta', '\theta (deg)'; ...
        'psi',   '\psi (deg)'    ...
    };

    figure('Name', ['DEBUG — ' condition_label], ...
           'NumberTitle', 'off', ...
           'Units', 'normalized', ...
           'Position', [0.05, 0.05, 0.90, 0.85]);

    sgtitle(['Full State Debug — ' condition_label], ...
            'FontSize', 13, 'FontWeight', 'bold');

    for i = 1:9
        subplot(3, 3, i);
        plot(sim_out.time, sim_out.(states{i,1}), 'LineWidth', 1.5);
        xlabel('Time (sec)');
        ylabel(states{i,2});
        title(states{i,2}, 'FontWeight', 'bold');
        grid on;  box on;
    end
end

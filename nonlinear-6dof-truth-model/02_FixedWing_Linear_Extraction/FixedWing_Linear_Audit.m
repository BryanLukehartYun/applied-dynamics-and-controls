%% ========================================================================
%  6-DOF Fixed-Wing Flight Simulator — Linearization Audit
%  ========================================================================
%  Author  : Bryan L, in conjunction with P.S and M.C
%             (co-authors anonymized at their discretion)
%  Course  : Flight Dynamics — MECE 410 (Project #5)
%  Date    : Circa Mid-2021
%  Purpose : Extracts a linearized state-space model from the nonlinear
%            truth model at trim, decouples it into longitudinal and
%            lateral-directional subsystems, performs modal stability
%            analysis, and produces overlay validation plots comparing the
%            linear approximation against the nonlinear truth model.
%
%  Linearization Method : MATLAB linmod() — numerical Jacobian evaluated
%                         at the trim operating point.
%  Constraint           : Cm_alphadot = 0 (set in Parameters file).
%
%  Longitudinal Sub-system  — States/Outputs : [VT, alpha, q, theta]^T
%                             Input           : [delta_e]
%  Lateral-Dir. Sub-system  — States/Outputs : [beta, p, r, phi]^T
%                             Inputs          : [delta_a, delta_r]^T
%
%  External Dependencies (treated as black-boxes):
%    - FixedWingSim_Parameters.m   : Aircraft & trim parameters (client input)
%    - FixedWing_Nonlinear_Audit   : 6-DOF nonlinear Simulink truth model
%    - FixedWing_Linear_Sim        : Linear state-space Simulink diagram
% =========================================================================

%% --- Workspace Initialization -------------------------------------------
clc;
clear;
close all;
tic;
% Navigate to script's actual saved location, not editor temp path
cd(fileparts(which('FixedWing_Linear_Audit.m')));
script_dir = pwd;
addpath(fullfile(script_dir, '..', '01_FixedWing_Nonlinear_Sim'));
run('FixedWingSim_Parameters.m');

%% --- Inertia Matrix Inverse (K) -----------------------------------------
% K = I^{-1} — passed to the Simulink diagram for angular acceleration.
K = inv([ Ixx, -Ixy, -Ixz;
         -Ixy,  Iyy, -Iyz;
         -Ixz, -Iyz,  Izz]);

%% ========================================================================
%  STEP 1 — LINEARIZATION AT TRIM (linmod Jacobian Extraction)
%  ========================================================================
%  linmod() numerically differentiates the nonlinear Simulink model at the
%  current workspace trim point to produce global Jacobian matrices:
%    dx/dt = A*x + B*u
%      y   = C*x + D*u
%
%  IMPORTANT: linmod must be called BEFORE any perturbed simulations.
%  The nonlinear sim is run at trim first to ensure Simulink is initialized
%  at the correct operating point before the Jacobian is computed.
% -------------------------------------------------------------------------

%  -- Initialize at trim (no perturbation) --------------------------------
e = de0;   a = da0;   r = dr0;
sim('FixedWing_Nonlinear_Audit');   % Warm-up run — establishes trim state

%  -- Extract global state-space matrices ----------------------------------
[A, B, C, D] = linmod('FixedWing_Nonlinear_Audit');

%% ========================================================================
%  STEP 2 — SUBSYSTEM DECOUPLING
%  ========================================================================
%  The global 8x8 system is manually partitioned into two decoupled 4x4
%  blocks using the known state ordering from the Simulink diagram:
%    Rows 1-4 : Longitudinal states  [VT, alpha, q, theta]
%    Rows 5-8 : Lateral-Dir. states  [beta, p, r, phi]
% -------------------------------------------------------------------------

%  -- Longitudinal sub-system (elevator input only) -----------------------
A_Long = A(1:4, 1:4);
B_Long = B(1:4, 1);
C_Long = C(1:4, 1:4);
D_Long = D(1:4, 1);

%  -- Lateral-Directional sub-system (aileron + rudder inputs) ------------
A_Lat  = A(5:8, 5:8);
B_Lat  = B(5:8, 2:3);
C_Lat  = C(5:8, 5:8);
D_Lat  = D(5:8, 2:3);

%% ========================================================================
%  STEP 3 — MODAL STABILITY ANALYSIS
%  ========================================================================
%  damp() computes eigenvalues, natural frequencies (wn), and damping
%  ratios (zeta) for each mode.
%
%  Longitudinal modes expected:
%    - Short Period : high-frequency, well-damped complex pair
%    - Phugoid      : low-frequency, lightly-damped complex pair
%
%  Lateral-Directional modes expected:
%    - Dutch Roll   : lightly-damped complex pair (beta/r oscillation)
%    - Roll         : fast real eigenvalue (p time constant)
%    - Spiral       : slow real eigenvalue (phi divergence/convergence)
% -------------------------------------------------------------------------

fprintf('\n==== LONGITUDINAL STATE-SPACE MODEL ====\n');
longss = ss(A_Long, B_Long, C_Long, D_Long)
fprintf('\n-- Longitudinal Modal Analysis (Short Period & Phugoid) --\n');
damp(longss)

fprintf('\n==== LATERAL-DIRECTIONAL STATE-SPACE MODEL ====\n');
latss  = ss(A_Lat,  B_Lat,  C_Lat,  D_Lat)
fprintf('\n-- Lateral-Dir. Modal Analysis (Dutch Roll, Roll, Spiral) --\n');
damp(latss)

%% ========================================================================
%  STEP 4 — TRANSFER FUNCTIONS (Longitudinal)
%  ========================================================================
%  Derive pitch rate and angle-of-attack transfer functions from the
%  longitudinal state-space model via ss2tf().
%  State order: [VT=row1, alpha=row2, q=row3, theta=row4]
% -------------------------------------------------------------------------
[num, den] = ss2tf(A_Long, B_Long, C_Long, D_Long);

fprintf('\n==== TRANSFER FUNCTIONS (Elevator Input) ====\n');
fprintf('  Pitch Rate    : q(s) / delta_e(s)\n');
qtf = tf(num(3,:), den)

fprintf('  Angle-of-Attack: alpha(s) / delta_e(s)\n');
atf = tf(num(2,:), den)

%% ========================================================================
%  STEP 5 — PERTURBED SIMULATIONS
%  ========================================================================
%  Three control perturbations are run back-to-back. Each stores both the
%  nonlinear truth model output and the linear state-space output.
%  Linear outputs are perturbation-relative; trim offsets are re-added
%  during plotting to reconstruct absolute state values.
% -------------------------------------------------------------------------

%  -- CONDITION A: Elevator Step (-0.5 deg) --------------------------------
e = de0 - (0.5 * d2r);   a = da0;          r = dr0;
e_lin = -0.5 * d2r;      a_lin = 0;        r_lin = 0;
sim_nl_elev  = sim('FixedWing_Nonlinear_Audit');
sim_lin_elev = sim('FixedWing_Linear_Sim');

%  -- CONDITION B: Aileron Step (-0.5 deg) ---------------------------------
e = de0;                  a = da0 - (0.5 * d2r);   r = dr0;
e_lin = 0;                a_lin = -0.5 * d2r;      r_lin = 0;
sim_nl_ail  = sim('FixedWing_Nonlinear_Audit');
sim_lin_ail = sim('FixedWing_Linear_Sim');

%  -- CONDITION C: Rudder Step (-2.0 deg) ----------------------------------
e = de0;                  a = da0;          r = dr0 - (2.0 * d2r);
e_lin = 0;                a_lin = 0;        r_lin = -2.0 * d2r;
sim_nl_rud  = sim('FixedWing_Nonlinear_Audit');
sim_lin_rud = sim('FixedWing_Linear_Sim');

%% ========================================================================
%  STEP 6 — OVERLAY VALIDATION FIGURES
%  ========================================================================
%  One 4x2 figure per condition. Each subplot overlays:
%    Solid line  — Nonlinear truth model
%    Dashed line — Linear state-space approximation (trim + perturbation)
%  Agreement indicates valid linearization; divergence marks nonlinearity.
%
%  Legend is placed once (subplot 2) to avoid clutter.
% -------------------------------------------------------------------------

%  Line style convention
nl_style  = '-';    nl_width  = 1.5;   % nonlinear : solid
lin_style = '--';   lin_width = 1.5;   % linear    : dashed

%  State definitions for the 4x2 overlay layout:
%    { NL field,  linear field,    trim offset,      y-label       }
overlay = { ...
    'VT',    'VT_lin',    Vt0,            'V_T (ft/s)';    ...
    'alpha', 'alpha_lin', alpha0 * r2d,   '\alpha (deg)';  ...
    'beta',  'beta_lin',  beta0  * r2d,   '\beta (deg)';   ...
    'p',     'p_lin',     p0     * r2d,   'p (deg/s)';     ...
    'q',     'q_lin',     q0     * r2d,   'q (deg/s)';     ...
    'r',     'r_lin',     r0     * r2d,   'r (deg/s)';     ...
    'phi',   'phi_lin',   phi0   * r2d,   '\phi (deg)';    ...
    'theta', 'theta_lin', theta0 * r2d,   '\theta (deg)'   ...
};

conditions = { ...
    sim_nl_elev, sim_lin_elev, 'Condition A — Elevator (\Deltae = -0.5°)'; ...
    sim_nl_ail,  sim_lin_ail,  'Condition B — Aileron  (\Deltaa = -0.5°)'; ...
    sim_nl_rud,  sim_lin_rud,  'Condition C — Rudder   (\Deltar = -2.0°)'  ...
};

for c = 1:3
    nl_out  = conditions{c, 1};
    lin_out = conditions{c, 2};
    fig_title = conditions{c, 3};

    figure('Name', ['Linear Audit — ' fig_title], ...
           'NumberTitle', 'off', ...
           'Units', 'normalized', ...
           'Position', [0.05, 0.05, 0.90, 0.85]);

    sgtitle(['Nonlinear vs. Linear State-Space — ' fig_title], ...
            'FontSize', 13, 'FontWeight', 'bold');

    for i = 1:8
        nl_field   = overlay{i, 1};
        lin_field  = overlay{i, 2};
        trim_offset = overlay{i, 3};
        ylbl       = overlay{i, 4};

        subplot(4, 2, i);
        hold on;

        plot(nl_out.time,  nl_out.(nl_field), ...
             nl_style,  'LineWidth', nl_width,  'DisplayName', 'Nonlinear');
        plot(lin_out.time_lin, lin_out.(lin_field) + trim_offset, ...
             lin_style, 'LineWidth', lin_width, 'DisplayName', 'State-Space');

        xlabel('Time (sec)');
        ylabel(ylbl);
        title(ylbl, 'FontWeight', 'bold');

        if i == 2   % Single legend placed on subplot 2 only
            legend('show', 'Location', 'best', 'FontSize', 8);
        end

        grid on;   box on;
    end
end

toc
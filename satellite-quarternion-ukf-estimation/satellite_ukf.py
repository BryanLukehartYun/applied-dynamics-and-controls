"""
Satellite UKF Wrapper
=====================
Connects the UKF to the satellite physics.

Key fixes vs. the original MATLAB version:
  - ukf_predict: covariance update was using elementwise multiply (diff.*diff)
    instead of the outer product (diff @ diff.T) — this was the main break.
  - sqrtm: use scipy.linalg.sqrtm which handles near-singular P better.
  - Symmetry enforcement on P at every step to prevent drift.
  - Full Monte Carlo support.
"""

import numpy as np
from scipy.linalg import sqrtm
import matplotlib.pyplot as plt

from satellite_dynamics_step import satellite_dynamics_step
from satellite_generator import generate_satellite_data


# ---------------------------------------------------------------------------
# UKF Core
# ---------------------------------------------------------------------------

def _sigma_weights(L: int, alpha: float = 1e-3, beta: float = 2.0, ki: float = 0.0):
    """Compute UKF sigma-point weights."""
    lam = alpha**2 * (L + ki) - L
    Wm = np.full(2 * L + 1, 1.0 / (2 * (L + lam)))
    Wm[0] = lam / (L + lam)
    Wc = Wm.copy()
    Wc[0] += (1 - alpha**2 + beta)
    return Wm, Wc, lam


def ukf_predict(x: np.ndarray, P: np.ndarray, f, Q: np.ndarray):
    """
    UKF Predict Step.

    Args:
        x : Prior state mean          (L,)
        P : Prior covariance          (L,L)
        f : Nonlinear dynamics func   callable, f(x) → x_next
        Q : Process noise covariance  (L,L)

    Returns:
        x_prior : Predicted mean       (L,)
        P_prior : Predicted covariance (L,L)
    """
    L = len(x)
    Wm, Wc, lam = _sigma_weights(L)

    # Sigma points — incorporate Q here for numerical stability
    P_aug = (P + P.T) / 2 + Q
    sP = np.real(sqrtm((L + lam) * P_aug))

    chi = np.column_stack([x] + [x + sP[:, i] for i in range(L)]
                               + [x - sP[:, i] for i in range(L)])  # (L, 2L+1)

    # Propagate sigma points through dynamics
    chi_prop = np.column_stack([f(chi[:, i]) for i in range(2 * L + 1)])

    # Weighted mean
    x_prior = chi_prop @ Wm

    # Weighted covariance  ← THE FIX: outer product, not elementwise
    P_prior = Q.copy()
    for i in range(2 * L + 1):
        d = chi_prop[:, i] - x_prior
        P_prior += Wc[i] * np.outer(d, d)

    return x_prior, P_prior


def ukf_update(x: np.ndarray, P: np.ndarray, z: np.ndarray, h, R: np.ndarray):
    """
    UKF Update Step.

    Args:
        x : Predicted state mean          (L,)
        P : Predicted covariance          (L,L)
        z : Measurement vector            (M,)
        h : Measurement function          callable, h(x) → z
        R : Measurement noise covariance  (M,M)

    Returns:
        x_post : Updated state mean       (L,)
        P_post : Updated covariance       (L,L)
    """
    L = len(x)
    Wm, Wc, lam = _sigma_weights(L)

    # Enforce symmetry + small regularization to keep P positive-definite
    P = (P + P.T) / 2 + np.eye(L) * 1e-9
    sP = np.real(sqrtm((L + lam) * P))

    chi = np.column_stack([x] + [x + sP[:, i] for i in range(L)]
                               + [x - sP[:, i] for i in range(L)])

    # Transform sigma points to measurement space
    Z_chi = np.column_stack([h(chi[:, i]) for i in range(2 * L + 1)])

    z_pred = Z_chi @ Wm

    # Innovation covariance S and cross-covariance Pxz
    M = len(z)
    S = R.copy()
    Pxz = np.zeros((L, M))
    for i in range(2 * L + 1):
        z_d = Z_chi[:, i] - z_pred
        x_d = chi[:, i] - x
        S   += Wc[i] * np.outer(z_d, z_d)
        Pxz += Wc[i] * np.outer(x_d, z_d)

    # Kalman gain and posterior
    K = Pxz @ np.linalg.inv(S)
    x_post = x + K @ (z - z_pred)
    P_post = P - K @ S @ K.T

    return x_post, P_post


# ---------------------------------------------------------------------------
# Single Run
# ---------------------------------------------------------------------------

def run_ukf(data: dict) -> np.ndarray:
    """
    Run one UKF pass over a generated dataset.

    Returns:
        x_est : Estimated states (7, N)
    """
    time   = data["time"]
    y_meas = data["y_meas"]
    J      = data["J"]
    dt     = data["dt"]
    R_star = data["R_star"]
    R_gyro = data["R_gyro"]
    N = len(time)

    # --- Noise Covariances ---
    Q = np.diag([1e-6, 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4])
    R = np.diag([R_star]*4 + [R_gyro]*3)
    P = np.eye(7) * 0.1

    # --- Initialization ---
    x_est = np.zeros((7, N))
    x_est[:, 0] = [1, 0, 0, 0, 0, 0, 0]   # Zero knowledge start

    f = lambda x: satellite_dynamics_step(x, np.zeros(3), dt, J)
    h = lambda x: x   # Direct observation (identity measurement model)

    for k in range(1, N):
        x_pred, P_pred = ukf_predict(x_est[:, k-1], P, f, Q)
        x_est[:, k], P = ukf_update(x_pred, P_pred, y_meas[:, k], h, R)

        # Normalize quaternion part to prevent drift
        x_est[0:4, k] /= np.linalg.norm(x_est[0:4, k])

    return x_est


# ---------------------------------------------------------------------------
# Monte Carlo
# ---------------------------------------------------------------------------

def run_monte_carlo(n_runs: int = 50, seed_base: int = 42) -> dict:
    """
    Run UKF estimation n_runs times with different random tip-off rates and noise.

    Returns:
        dict with per-run errors and summary stats.
    """
    rng = np.random.default_rng(seed_base)
    errors_q = []     # RMS quaternion error per run
    errors_w = []     # RMS angular velocity error per run

    print(f"Running Monte Carlo ({n_runs} trials)...")
    for i in range(n_runs):
        # Randomize initial tip-off rate ±0.3 rad/s per axis
        omega0 = rng.uniform(-0.3, 0.3, size=3)
        x0 = np.array([1.0, 0.0, 0.0, 0.0, *omega0])

        data = generate_satellite_data(x0=x0, seed=int(rng.integers(0, 99999)))
        x_est = run_ukf(data)
        x_truth = data["x_truth"]

        err_q = np.sqrt(np.mean((x_est[0:4, :] - x_truth[0:4, :])**2))
        err_w = np.sqrt(np.mean((x_est[4:7, :] - x_truth[4:7, :])**2))
        errors_q.append(err_q)
        errors_w.append(err_w)

        if (i + 1) % 10 == 0:
            print(f"  Completed {i+1}/{n_runs}")

    print("Monte Carlo complete.")
    return {
        "errors_q": np.array(errors_q),
        "errors_w": np.array(errors_w),
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_single_run(data: dict, x_est: np.ndarray):
    """Plot truth vs estimate for a single run."""
    time    = data["time"]
    x_truth = data["x_truth"]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7))

    ax = axes[0]
    for i, label in enumerate(["q1", "q2", "q3", "q4"]):
        ax.plot(time, x_truth[i, :], "k", lw=1.5, alpha=0.6)
        ax.plot(time, x_est[i, :], "--", lw=1, label=f"{label} est")
    ax.set_title("Attitude (Quaternions): Truth (black) vs UKF Estimate (dashed)")
    ax.set_ylabel("Value")
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[1]
    labels_w = ["wx", "wy", "wz"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    for i in range(3):
        ax.plot(time, x_truth[4+i, :], "k", lw=1.5, alpha=0.6)
        ax.plot(time, x_est[4+i, :], "--", color=colors[i], lw=1, label=f"{labels_w[i]} est")
    ax.set_title("Angular Velocity (rad/s): Truth (black) vs UKF Estimate (dashed)")
    ax.set_ylabel("rad/s")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout()
    plt.savefig("ukf_single_run.png", dpi=150)
    plt.show()
    print("Plot saved to ukf_single_run.png")


def plot_monte_carlo(mc_results: dict):
    """Plot Monte Carlo RMS error distributions."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    for ax, errors, title, unit in zip(
        axes,
        [mc_results["errors_q"], mc_results["errors_w"]],
        ["Quaternion RMS Error", "Angular Velocity RMS Error"],
        ["[-]", "[rad/s]"],
    ):
        ax.hist(errors, bins=15, edgecolor="black", color="steelblue", alpha=0.8)
        ax.axvline(np.mean(errors), color="red", lw=2, linestyle="--", label=f"Mean: {np.mean(errors):.4f}")
        ax.set_title(f"Monte Carlo {title}")
        ax.set_xlabel(f"RMS Error {unit}")
        ax.set_ylabel("Count")
        ax.legend()

    plt.tight_layout()
    plt.savefig("ukf_monte_carlo.png", dpi=150)
    plt.show()
    print("Monte Carlo plot saved to ukf_monte_carlo.png")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # --- Single Run ---
    print("=== Single Run ===")
    data = generate_satellite_data(seed=42)
    x_est = run_ukf(data)
    plot_single_run(data, x_est)

    # --- Monte Carlo ---
    print("\n=== Monte Carlo (50 runs) ===")
    mc = run_monte_carlo(n_runs=50)
    plot_monte_carlo(mc)
    print(f"\nQuaternion  RMS Error — Mean: {mc['errors_q'].mean():.5f}, Std: {mc['errors_q'].std():.5f}")
    print(f"Ang. Vel.   RMS Error — Mean: {mc['errors_w'].mean():.5f}, Std: {mc['errors_w'].std():.5f}")

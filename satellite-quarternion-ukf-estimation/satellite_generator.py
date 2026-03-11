import numpy as np
from satellite_dynamics_step import satellite_dynamics_step


def generate_satellite_data(
    dt: float = 0.1,
    t_end: float = 100.0,
    J: np.ndarray = None,
    x0: np.ndarray = None,
    R_star: float = 0.005,
    R_gyro: float = 0.01,
    seed: int = None,
) -> dict:
    """
    Generate synthetic satellite truth + noisy sensor measurements.

    Args:
        dt     : Timestep (s), default 0.1 → 10 Hz
        t_end  : Simulation duration (s)
        J      : 3×3 inertia matrix. Defaults to diag([10, 8, 5])
        x0     : Initial state [q1..q4, wx..wz]. Defaults to identity quat + tip-off rates
        R_star : Star tracker noise std dev (quaternion units)
        R_gyro : Gyro noise std dev (rad/s)
        seed   : Optional random seed for reproducibility

    Returns:
        dict with keys: time, x_truth, y_meas, y_star, y_gyro, R_star, R_gyro, J, dt
    """
    if seed is not None:
        np.random.seed(seed)

    if J is None:
        J = np.diag([10.0, 8.0, 5.0])

    if x0 is None:
        x0 = np.array([1.0, 0.0, 0.0, 0.0,   # Identity quaternion
                        0.1, -0.2, 0.05])       # Tip-off angular rates (rad/s)

    time = np.arange(0, t_end + dt, dt)
    N = len(time)

    # --- Truth Propagation ---
    x_truth = np.zeros((7, N))
    x_truth[:, 0] = x0

    M_ctrl = np.zeros(3)   # Passive tumble — no control torques

    for k in range(N - 1):
        x_truth[:, k + 1] = satellite_dynamics_step(x_truth[:, k], M_ctrl, dt, J)

    # --- Noisy Measurements ---
    y_star = x_truth[0:4, :] + R_star * np.random.randn(4, N)
    y_gyro = x_truth[4:7, :] + R_gyro * np.random.randn(3, N)
    y_meas = np.vstack([y_star, y_gyro])   # Combined 7×N measurement matrix

    print(f'Satellite "Truth" and Noisy Telemetry generated. ({N} timesteps)')

    return {
        "time":    time,
        "x_truth": x_truth,
        "y_meas":  y_meas,
        "y_star":  y_star,
        "y_gyro":  y_gyro,
        "R_star":  R_star,
        "R_gyro":  R_gyro,
        "J":       J,
        "dt":      dt,
    }

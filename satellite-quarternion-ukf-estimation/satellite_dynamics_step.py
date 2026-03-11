import numpy as np


def satellite_dynamics_step(x: np.ndarray, M_ctrl: np.ndarray, dt: float, J: np.ndarray) -> np.ndarray:
    """
    One step of satellite rigid-body dynamics.

    Args:
        x      : State vector [q1, q2, q3, q4, wx, wy, wz] (7,)
        M_ctrl : Control torque [Mx, My, Mz]               (3,)
        dt     : Timestep (seconds)
        J      : Inertia matrix                             (3,3)

    Returns:
        x_next : Updated state vector                       (7,)
    """
    q = x[0:4]
    omega = x[4:7]

    # --- 1. Quaternion Kinematics (dq/dt = 0.5 * Omega(omega) * q) ---
    # Big-Omega matrix for quaternion propagation
    wx, wy, wz = omega
    Omega_mat = np.array([
        [ 0,   wz, -wy,  wx],
        [-wz,  0,   wx,  wy],
        [ wy, -wx,  0,   wz],
        [-wx, -wy, -wz,  0 ]
    ])
    dq = 0.5 * Omega_mat @ q

    # --- 2. Euler Rotational Dynamics ---
    # J * d(omega)/dt = M_ctrl - omega x (J * omega)
    d_omega = np.linalg.solve(J, M_ctrl - np.cross(omega, J @ omega))

    # --- 3. Euler Integration ---
    q_next = q + dq * dt
    q_next = q_next / np.linalg.norm(q_next)   # Always re-normalize!

    omega_next = omega + d_omega * dt

    return np.concatenate([q_next, omega_next])

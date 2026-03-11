# Applied Dynamics and Controls

**Author:** Bryan Lukehart-Yun  
**Toolchain:** MATLAB / Simulink / Python  
**Focus:** Aerospace GNC — Flight Dynamics, State Estimation, and Control Systems

---

## Overview

This repository is a centralized portfolio of applied dynamics and controls work spanning classical flight mechanics and modern nonlinear control theory. Each module demonstrates a distinct layer of the GNC engineering workflow — from high-fidelity truth model construction to linearization, modal analysis, and validation.

Current modules are aerospace-focused, reflecting coursework in flight dynamics and control systems. The repository is actively expanding.

---

## Repository Structure

```
Applied-dynamics-and-controls/
│
├── README.md                          ← You are here
│
├── nonlinear-6dof-truth-model/        ← Completed | MATLAB + Simulink (Treat as Black Box)
├── satellite-quaternion-ukf/          ← Stable | Python
├── interplanetary-mission-design/     ← Active | MATLAB -> Python (Migrating)
├── orbital-mechanics-solver/          ← Planned
└── sysid-python-work/                 ← Planned
```

---

## Modules

### `nonlinear-6dof-truth-model` — Completed | No active development

A two-part fixed-wing flight dynamics study built around a Six-Degree-of-Freedom nonlinear simulation.

**Part 01** constructs the nonlinear truth model and verifies trim stability across four control perturbation conditions (trim hold, elevator, aileron, rudder).

**Part 02** linearizes the truth model at trim using `linmod()`, decouples the result into longitudinal and lateral-directional subsystems, performs modal stability analysis (Short Period, Phugoid, Dutch Roll, Roll, Spiral), and validates the linear approximation against the truth model through overlay comparison plots.

→ [See full technical README](nonlinear-6dof-truth-model/README.md)

---

### `satellite-quaternion-ukf` — Stable

Satellite attitude estimation using an Unscented Kalman Filter on a tumbling rigid-body spacecraft. Models 3D attitude dynamics via quaternion kinematics and Euler's equations, fusing noisy star tracker and gyroscope measurements to recover true attitude and angular velocity. Robustness validated via Monte Carlo simulation across randomized initial tumble rates.

**Key Results:** Quaternion RMS error mean of 0.0013 | Angular velocity RMS mean of 0.0057 rad/s | 100% Monte Carlo convergence.

→ [See full technical README](satellite-quarternion-ukf-estimation/README.md)


---

### `interplanetary-mission-design` — Active
Constrained optimization of a non-Hohmann Earth-to-Saturn transfer trajectory.
Five design variables, symbolically-derived ΔV cost function, MATLAB problem-based
optimizer. Python migration and interactive visualization planned.

→ [See full technical README](interplanetary-mission-design/README.md)

---

### `orbital-mechanics-solver` — Planned

---

### `sysid-python-work` — Planned

---

## Background

This repository exists as a focused aerospace GNC portfolio. 
* The flight dynamics modules reflect work from **MECE-410: Flight Dynamics** (Spring 2021), the predecessor to Graduate Course of the same name, at RIT, refactored and documented to professional standards.
* The Satellite Quarternion UKF estimation project reflects a desire to understand how such work into satellite and 6-DOF might occur.

For nonlinear GNC work on soft actuator systems (NLARX System ID, UKF state estimation, NMPC), see the companion repository:  

→ [Nonlinear-Actuators-GNC-Robotics-Framework](https://github.com/BryanLukehartYun/Nonlinear-Actuators-GNC-Robotics-Framework)


---

# Academic Integrity Notice

Some modules in this repository are derived from coursework completed at the Rochester Institute of Technology. **If you are a student, you are expected to comply with your home institution's academic integrity and honor code policies before referencing or adapting any material here.** This repository is intended as a professional portfolio demonstration, not a resource for academic submission.

The Simulink diagrams and parameter files in the flight dynamics modules are instructor-provided scaffolding. The engineering judgment, code architecture, and documentation are original work.

---

## Licensing

- **Software & Code:** All MATLAB and Python source files are licensed under the **MIT License** — free to use, modify, and distribute with attribution.
- **Technical Analysis:** Reports, figures, and design-decision narratives (including the interplanetary trajectory optimization) are licensed under **CC BY 4.0** — you may share and adapt with attribution, including for commercial purposes.

Full license text: [MIT](LICENSE) | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)

---
## Contact

| Inquiry | Contact |
|---|---|
| Recruitment & Professional | bryan.lukehartyun@gmail.com |
| Technical & Repository | Open a GitHub Issue |

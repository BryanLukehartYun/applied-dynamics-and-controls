# Interplanetary Mission Design — Earth to Saturn Trajectory Optimization

Constrained optimization of a non-Hohmann elliptical transfer from an Earth parking orbit to a Saturn capture orbit. The total delta-V required for the mission is treated as a cost function over five design variables, and minimized numerically using MATLAB's problem-based optimization framework with symbolically-derived gradients.

---

## Problem Setup

A spacecraft departs from an elliptical parking orbit around Earth on a non-Hohmann transfer ellipse to Saturn. At arrival, it is captured into an elliptical orbit around Saturn. The mission architecture involves three trajectory phases:

1. **Hyperbolic escape** from Earth parking orbit
2. **Heliocentric transfer ellipse** — non-Hohmann, departure velocity vector perpendicular to the Sun
3. **Hyperbolic capture** into Saturn orbit

**Assumptions:**
- Earth and Saturn are in circular orbits around the Sun
- Departure velocity vector is perpendicular to the Sun at Earth
- I_sp = 290 s (solid rocket propellant baseline)

---

## Design Variables

| Variable | Definition | Bounds |
|----------|-----------|--------|
| x₁ | Periapsis altitude of Earth parking orbit (km) | ≥ 1,000 |
| x₂ | Apoapsis altitude of Earth parking orbit (km) | [1,000 — 3,600] |
| x₃ | Periapsis altitude of Saturn capture orbit (km) | ≥ 283,000 (above widest ring) |
| x₄ | Apoapsis altitude of Saturn capture orbit (km) | [283,000 — 54,800,000] (sphere of influence) |
| x₅ | True anomaly angle at Saturn encounter (deg) | [0 — 180] |

Bound rationale: x₁/x₂ constrained to Medium Earth Orbit range. x₃ lower bound set above Saturn's outermost ring (~283,000 km). x₄ upper bound is Saturn's sphere of influence. x₅ physically cannot exceed 180°.

---

## Cost Function

Total delta-V is the sum of the departure burn (Earth escape) and arrival burn (Saturn capture), expressed symbolically as a function of the five design variables:

```
ΔV_total(x₁,x₂,x₃,x₄,x₅) =

((- x2 + x1)/(2·r_E + x2 + x1) - 1)·(-mu_E/(((- x2 + x1)²/(2·r_E + x1 + x2)² - 1)·(r_E + x2/2 + x1/2)))^(1/2)
+ (|(mu_Sun/R_E)^(1/2) + (mu_Sun·((R_E - R_Sat)/(R_E - cos((x5·π)/180)·R_Sat) - 1))/(-R_E·mu_Sun·((R_E - R_Sat)/(R_E - R_Sat·cos((x5·π)/180)) - 1))^(1/2)|² + (2·mu_E)/(r_E + x1))^(1/2)
- (-(mu_Sat·((- x4 + x3)/(2·r_Sat + x4 + x3) - 1))/(r_Sat + x3))^(1/2)
+ ((2·mu_Sat)/(r_Sat + x3) + |(mu_Sun/R_Sat)^(1/2) + (mu_Sun·((cos((π·x5)/180)·(R_E - R_Sat))/(R_E - cos((x5·π)/180)·R_Sat) - 1))/(-R_E·mu_Sun·((R_E - R_Sat)/(R_E - R_Sat·cos((x5·π)/180)) - 1))^(1/2)|² + |mu_Sun·sin((π·x5)/180)·(R_E - R_Sat)|²/(|R_E - R_Sat·cos((x5·π)/180)|²·|R_E·mu_Sun·((R_E - R_Sat)/(R_E - cos((x5·π)/180)·R_Sat) - 1)|))^(1/2)
```

Propellant mass fraction from Tsiolkovsky:

$$\frac{\Delta m}{m} = 1 - e^{\frac{-\Delta V_{total}}{I_{sp} \cdot g_0}}$$

The symbolic expression was derived in MATLAB's Symbolic Math Toolbox and passed directly to `fcn2optimexpr` for gradient-based numerical optimization. See [`EarthToSaturn_Symbolic.m`](EarthToSaturn_Symbolic.m) for the full derivation.

---

## Results

| Variable | Definition | Optimal Value |
|----------|-----------|---------------|
| x₁ | Periapsis altitude — Earth parking orbit | 1,000 km |
| x₂ | Apoapsis altitude — Earth parking orbit | 3,600 km |
| x₃ | Periapsis altitude — Saturn capture orbit | 283,000.7 km |
| x₄ | Apoapsis altitude — Saturn capture orbit | 54,799,500 km |
| x₅ | True anomaly at encounter | 179.99° |
| ΔV | Total delta-V | **7.7599 km/s** |
| Δm/m | Propellant fraction | **93.463%** |

x₁ and x₂ converge to their bounds — consistent with the physics: minimizing departure ΔV favors the lowest feasible parking orbit periapsis and the smallest apoapsis. x₅ converges near 180°, approaching a near-Hohmann geometry as the optimizer seeks the most efficient transfer angle within the non-Hohmann constraint set.

---

## Known Limitations

- MATLAB's optimizer drives x₁ to its lower bound due to the ΔV gradient structure — the bound itself becomes the active constraint
- Saturn arrival orbit initial conditions are idealized; no Saturn atmospheric or ring perturbations modeled
- Time-of-flight calculation has a known inconsistency for near-180° true anomaly values and should be treated as approximate

---
## Roadmap

**v1.0 — MATLAB Baseline** *(current)*
Constrained 5-variable delta-V optimization with symbolic cost function derivation.

**v1.1 — Python Migration**
Full port to Python using `scipy.optimize`. This and all subsequent versions
are independent extensions beyond the original coursework.

**v1.2 — Interactive Visualization**
Matplotlib/Plotly GUI showing transfer ellipse, parking orbits, hyperbolic
departure/arrival trajectories, and delta-V breakdown interactively.

**v1.3 — Extended Optimization**
Multi-objective optimization — Pareto front across delta-V, time-of-flight,
and propellant fraction. Propellant sensitivity analysis across I_sp values.

**v1.9x — Kerbal Space Program Validation** *(long-term)*
Re-enact the optimized trajectory in KSP as a sanity check against
a physics sandbox. Because why not.

---

## Files

| File | Description |
|------|-------------|
| `EarthToSaturn_Optimization.m` | Main optimization script — 5-variable constrained ΔV minimization |
| `EarthToSaturn_Symbolic.m` | Symbolic derivation of the cost function via MATLAB Symbolic Math Toolbox |
| `assets/InterplanetaryDraft.pdf` | Original group presentation (May 2023) |

---

## References

- Curtis, H.D. (2019). *Orbital Mechanics for Engineering Students*, 4th Ed. Elsevier.
- MATLAB Optimization Toolbox — Problem-Based Approach Documentation

---

> **Source:** Adapted from MECE-611 Optimal Design final project, RIT (May 2023).
> Original submission preserved in [`assets/`](assets/). NOTE: As of March 09, the pdf upload is on hold. However all relevant information are listed in readme. 
> Co-developed with Group 13 and all authors anonymized at author's discretion.

---

*Part of the [Applied-Dynamics-and-Controls](../) repository.*
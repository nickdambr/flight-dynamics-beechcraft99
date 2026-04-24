# Flight Dynamics Analysis of the Beechcraft 99

**Longitudinal and lateral-directional stability, reduced-order modeling, and frequency response of a light turboprop aircraft.**

Course project — *Dinamica del Volo* (Flight Dynamics), Prof. Guido De Matteis
Undergraduate program in Aeronautical Engineering, Sapienza Università di Roma, A.Y. 2024/25
Group 6: Elisa Jacopucci, **Niccolò D'Ambrosio**, Francesco Daniele, Matteo Grippo

---

## Overview

This project characterizes the dynamic behavior of the **Beechcraft 99** twin-turboprop aircraft in a subsonic, level-flight trim condition ($V = 95\ \text{m/s}$, $H = 270\ \text{m}$). Starting from the nonlinear aircraft model provided by the [Airlib library](libs/airlib/), we compute the trim point, linearize the system, and analyze the resulting state-space models both numerically and through classical reduced-order approximations. The work closes with a frequency-domain study of the response to elevator inputs.

The complete **technical report (55 pages, in Italian)** is available in [report/Ldg_1_Gruppo_6_2024_25.pdf](report/Ldg_1_Gruppo_6_2024_25.pdf).

## What the project covers

1. **Trim computation** — equilibrium state and control deflections via the `air3m` Newton-based trim routine.
2. **Numerical linearization** — state-space matrices of the longitudinal and lateral-directional subsystems obtained via Simulink *Model Linearizer* (block-wise numerical perturbation).
3. **Modal analysis** — eigenvalues / eigenvectors of both subsystems, identification of the five canonical modes:
   - **Longitudinal**: phugoid, short period, density-gradient mode
   - **Lateral-directional**: spiral, roll, dutch roll
4. **Reduced-order models** — Lanchester's classical model and multiple physical-approximation models for each mode; comparison against the exact eigenstructure, including the density-gradient correction factor.
5. **Lateral-directional stability region** — characterization of the stability envelope in the $(L'_\beta, N'_\beta)$ plane via the Routh discriminant, locating the Beechcraft 99 within it.
6. **Frequency response** — transfer functions from elevator input $\delta_E$ to $V$, $\alpha$, $\theta$; Bode plots, bandwidth analysis, effect of neglecting the density gradient.

## Key results

| Mode | $\omega_n$ [rad/s] | $\zeta$ | Period [s] |
|---|---|---|---|
| Phugoid | 0.124 | 0.094 | 50.6 |
| Short period | 6.08 | 0.606 | 1.04 |
| Dutch roll | 2.26 | 0.191 | 2.77 |
| Roll (τ) | — | — | 0.18 |
| Spiral (τ) | — | — | 28.5 |

The Beechcraft 99 sits well inside the stable region of both the spiral and dutch-roll constraints in the $(L'_\beta, N'_\beta)$ plane, consistent with certification expectations for a general-aviation turboprop.

All figures — pole–zero maps, Argand diagrams for every mode, lateral-directional stability envelope in the $(L'_\beta, N'_\beta)$ plane, Bode plots for $\Delta V / \Delta \delta_E$, $\Delta \alpha / \Delta \delta_E$, $\Delta \theta / \Delta \delta_E$ — are included in the [technical report](report/Ldg_1_Gruppo_6_2024_25.pdf) with full discussion.

## Repository layout

```
.
├── src/                    — authored analysis code and Simulink models
│   ├── flight_dynamics_analysis.m    main script: trim, linearization, modal analysis
│   ├── bode_plots.m                  frequency-response study (my personal contribution)
│   ├── expand_roots_into_factors.m   helper: factored representation of transfer functions
│   ├── export_figures.m              convenience script: runs both analyses and saves PNGs
│   ├── beechcraft99_longitudinal.slx Simulink model, longitudinal linearization setup
│   ├── beechcraft99_lateral.slx      Simulink model, lateral-directional linearization setup
│   ├── linsysLONG_beechcraft99.mat   linearized longitudinal state-space (saved from Simulink)
│   ├── linsysLAT_beechcraft99.mat    linearized lateral-directional state-space
│   └── bode.nb                       Mathematica notebook (polynomial factoring)
├── libs/airlib/            — third-party Simulink Aircraft Library by G. Campa (BSD license)
└── report/                 — final technical report (PDF)
```

Running `src/export_figures.m` in MATLAB regenerates all plots as PNGs under a `figures/` directory (not tracked — the report already contains them).

## How to run

Requirements: **MATLAB R2020b or later** with Simulink, Control System Toolbox, and Symbolic Math Toolbox.

```matlab
cd src
addpath(genpath('../libs/airlib'))
flight_dynamics_analysis   % trim, linearization, modal analysis, Argand diagrams
bode_plots                 % frequency-response analysis
export_figures             % optional: regenerate the PNGs in ../figures/
```

The scripts reproduce all figures and tables from the [report](report/Ldg_1_Gruppo_6_2024_25.pdf).

## Team contributions

The work was split across the four group members as follows (full activity log in Appendix A of the [report](report/Ldg_1_Gruppo_6_2024_25.pdf)):

| Member | Matricola | Primary responsibility | Report |
|---|---|---|---|
| Elisa Jacopucci | 1987922 | Trim-point computation and Simulink-based linearization of the nonlinear model | Ch. 2–3 |
| **Niccolò D'Ambrosio** (group coordinator) | **2001431** | **Frequency-response analysis: symbolic transfer functions and Bode plots** | **Ch. 6** |
| Francesco Daniele | 2008421 | Dynamic-characteristics analysis: poles, eigenvalues, eigenvectors, Argand diagrams | Ch. 4 |
| Matteo Grippo | 2216223 | Reduced-order models and consistency check against the exact eigenstructure | Ch. 5 |

All members contributed to: Simulink model assembly, validation of results, report writing and review.

### My personal contribution in detail

I was responsible for the **frequency-response analysis** (Chapter 6 of the report):
- derivation of the symbolic transfer functions $\Delta V / \Delta \delta_E$, $\Delta \alpha / \Delta \delta_E$, $\Delta \theta / \Delta \delta_E$ in factored form;
- Bode plots and interpretation (bandwidth, resonant peaks at phugoid and short-period frequencies, phase behavior of a non-minimum-phase aircraft);
- comparative study of the longitudinal model **with vs. without the density-gradient term**, showing how it alters the steady-state gain and long-term stability of the velocity transfer function;
- characterization of the lateral-directional stability envelope in the $(L'_\beta, N'_\beta)$ plane via the Routh discriminant (§5.2.5, jointly with E. Jacopucci).

Implementation: [src/bode_plots.m](src/bode_plots.m) and the helper [src/expand_roots_into_factors.m](src/expand_roots_into_factors.m).

## References

- B. Etkin, L. D. Reid, *Dynamics of Flight: Stability and Control*, 3rd ed., Wiley, 1996.
- M. V. Cook, *Flight Dynamics Principles*, 2nd ed., Elsevier, 2007.
- P. Bolzern, R. Scattolini, N. Schiavoni, *Fondamenti di controlli automatici*, 4th ed., McGraw-Hill, 2015.
- MathWorks, *Simulink Control Design User's Guide*, R2015b.
- G. Campa, [Simulink Aircraft Library (Airlib)](https://www.mathworks.com/matlabcentral/fileexchange/3019-simulink-aircraft-library), MATLAB File Exchange.

## License

The analysis code in [src/](src/) is released for academic reference.
The [Airlib](libs/airlib/) library is redistributed under its original BSD license — see [libs/airlib/license.txt](libs/airlib/license.txt).

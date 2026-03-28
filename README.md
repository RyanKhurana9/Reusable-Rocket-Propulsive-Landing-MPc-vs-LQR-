# 🚀 Rocket Propulsive Landing — MPC vs LQR

> 2D closed-loop simulation of a reusable rocket performing a propulsive vertical landing using Model Predictive Control and Linear Quadratic Regulator.

![Python](https://img.shields.io/badge/Python-3.8+-blue?style=flat-square&logo=python)
![NumPy](https://img.shields.io/badge/NumPy-1.21+-013243?style=flat-square&logo=numpy)
![SciPy](https://img.shields.io/badge/SciPy-1.7+-8CAAE6?style=flat-square&logo=scipy)
![Matplotlib](https://img.shields.io/badge/Matplotlib-3.4+-11557C?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)

---

## Overview

A reusable rocket starts 80 m above a landing pad, 50 m to the side, descending at 4 m/s, drifting sideways, and tilted 10°. Two control architectures must bring it to a soft, upright, zero-velocity landing at the origin.

| Controller | Concept | Constraints |
|---|---|---|
| **LQR** | Solves DARE offline once → fixed gain `u = −Ks` | ✗ Ignores limits |
| **MPC** | Solves constrained QP online every timestep | ✓ Enforces all limits |

The nonlinear plant is always propagated with 4th-order Runge-Kutta. Both controllers are designed on the linearised model and validated on full nonlinear dynamics.

---

## Results

| Metric | LQR | MPC |
|---|---|---|
| Landed | ✓ | ✓ |
| Touchdown vz | 1.31 m/s | **0.00 m/s** |
| Lateral error | 0.70 m | 0.37 m |
| Peak tilt | 11.4° | 11.4° |
| Tilt constraint violated | No | No |
| Gimbal constraint violated | No | No |

MPC achieves near-zero touchdown velocity because the altitude constraint `z ≥ 0` becomes active in the 1.5-second prediction window — the solver plans to bleed off descent rate before the ground arrives. LQR cannot do this because it has no look-ahead.

---

## Physics

### State vector

```
s = [ x,   z,   vx,   vz,   θ,   ω ]
      │    │    │     │     │    └── angular rate       [rad/s]
      │    │    │     │     └─────── body tilt          [rad]
      │    │    │     └──────────── vertical velocity   [m/s]
      │    │    └────────────────── horizontal velocity [m/s]
      │    └─────────────────────── altitude            [m]
      └──────────────────────────── lateral position    [m]
```

### Control inputs

```
u = [ ΔT,    δ  ]
      │       └── gimbal angle  (nozzle swivel, ±20°)
      └────────── thrust perturbation from hover  (T - T_hover)
```

**θ vs δ — a critical distinction:**
- `θ` is a *state* — the body tilt angle. Consequence of past torques. Cannot be changed directly.
- `δ` is an *input* — the nozzle deflection. Commanded by the controller each timestep.

The torque equation `θ̈ = −(T·L/Izz)·sin(δ)` contains only `δ`, not `θ`. A tilted rocket with zero gimbal produces zero self-correcting torque — it will not return upright on its own.

### Equations of motion (nonlinear)

```
ẍ  = (T/m) · sin(θ + δ)           horizontal acceleration
z̈  = (T/m) · cos(θ + δ) − g       vertical acceleration
θ̈  = −(T·L / Izz) · sin(δ)       angular acceleration
```

Angle `θ` is measured from the vertical axis, so `sin` gives the horizontal component and `cos` gives the vertical component.

---

## Control Design Pipeline

```
Nonlinear ODE
      │
      ▼
Linearise around hover equilibrium (small-angle approximation)
      │
      ▼
ṡ = Ac·s + Bc·u   (continuous-time linear model)
      │
      ▼
ZOH Discretisation via matrix exponential
      │
      ▼
s[k+1] = Ad·s[k] + Bd·u[k]   (discrete-time)
      │
      ├──────────────────────────────────────────┐
      ▼                                          ▼
   LQR                                        MPC
   ───                                        ───
   Solve DARE for P                           Build Sx, Su prediction matrices
   K = (R + BᵀPB)⁻¹ BᵀPA                    Formulate constrained QP
   u* = −K·s                                 Solve online with SLSQP
   One matrix multiply at runtime             Apply first action, recede horizon
```

### Linearisation

Taylor expansion around hover (`s* = 0`, `T* = T_hover`, `δ* = 0`):

```
v̇x ≈  g·(θ + δ)               tilt drives horizontal acceleration
v̇z ≈  ΔT / m                  thrust perturbation drives vertical acceleration
ω̇  ≈ −(T_hover·L / Izz)·δ    gimbal drives angular acceleration
```

### Discretisation

Both `Ad` and `Bd` computed in one matrix exponential:

```python
M_e = [[Ac·dt,  Bc·dt],
       [  0,      0  ]]

expm(M_e) = [[Ad,  Bd],
             [ 0,   I ]]
```

### Controllability

```
C = [Bd,  Ad·Bd,  Ad²·Bd,  Ad³·Bd,  Ad⁴·Bd,  Ad⁵·Bd]
rank(C) = 6 = n   →   FULLY CONTROLLABLE ✓
```

Open-loop eigenvalues are all exactly 1.0 (marginally stable — integrators). Active control required.

---

## LQR

**Cost function:**
```
J = Σ [ sₖᵀ Q sₖ  +  uₖᵀ R uₖ ]    (infinite horizon)
```

**Weight matrices:**
```python
Q = diag([0.02,   # x    lateral position
           0.20,   # z    altitude
           0.05,   # vx   horizontal velocity
           2.00,   # vz   vertical velocity
         300.0,    # θ    tilt  ← large: keep upright
          30.0])   # ω    angular rate

R = diag([1e-5,    # ΔT   thrust (near-free)
         5000.0])  # δ    gimbal (expensive → stays within limits)
```

**Solution:**
```
DARE:  P = Adᵀ P Ad − Adᵀ P Bd (R + Bdᵀ P Bd)⁻¹ Bdᵀ P Ad + Q
Gain:  K = (R + Bdᵀ P Bd)⁻¹ Bdᵀ P Ad
Law:   u* = −K·s
```

**Closed-loop eigenvalues** (all < 1 → stable):
```
|λ₁| = |λ₂| = 0.994   (slowest mode, ~16s time constant)
|λ₃| = |λ₄| = 0.979
|λ₅| = |λ₆| = 0.952   (fastest mode, ~2s time constant)
```

---

## MPC

**Cost function:**
```
J = Σ [ sₖᵀ Q sₖ  +  uₖᵀ R uₖ ]  +  sₙᵀ P sₙ    (N=15 steps, terminal cost P from LQR)
```

**Constraints:**
```
T_min  ≤  T_hover + ΔTₖ  ≤  T_max    thrust limits
|δₖ|   ≤  δ_max                       gimbal limits  ±20°
|θₖ|   ≤  θ_max                       tilt safety    ±25°
 zₖ    ≥  0                            no underground
```

**Condensed QP (offline precomputation):**
```
Predicted states:   Ŝ = Sx · s₀  +  Su · U

Sx ∈ R^{96×6}    free response  (powers of Ad)
Su ∈ R^{96×30}   forced response (lower-triangular, causal)

QP:   min  ½ Uᵀ H U  +  fᵀ U    s.t.  G·U ≤ h(s₀)

H = 2(Suᵀ Q̄ Su + R̄)    constant, precomputed once
f = 2 Suᵀ Q̄ Sx s₀      recomputed each timestep
```

H is positive definite → strictly convex QP → unique global minimum guaranteed.

**Online solve:** `scipy.optimize.minimize(..., method='SLSQP')` with analytic gradient and warm-start from previous solution.

---

## Project Structure

```
.
├── rocket_landing_mpc_lqr.py   # standalone script
├── rocket_landing.ipynb        # Jupyter notebook (13 cells)
├── rocket_landing_mpc_lqr.png  # output figure
└── README.md
```

### Notebook cell map

| Cell | Contents |
|---|---|
| 1 | Imports |
| 2 | Physical parameters |
| 3 | Nonlinear ODE + RK4 + hover sanity check |
| 4 | Linearisation + ZOH discretisation + controllability |
| 5 | LQR — DARE, gain K, eigenvalue stability check |
| 6 | MPC prediction matrices Sx, Su + sparsity plot |
| 7 | QP formulation — H, f, G, h + positive-definite check |
| 8 | Online MPC solver + single-step test |
| 9 | Closed-loop simulation loop |
| 10 | Printed results summary |
| 11 | 8-panel matplotlib figure |
| 12 | Experiment cell — change initial conditions |
| 13 | Extensions and further exploration |

---

## Installation

```bash
git clone https://github.com/your-username/rocket-landing-mpc-lqr.git
cd rocket-landing-mpc-lqr
pip install numpy scipy matplotlib
```

No additional packages required beyond the standard scientific Python stack.

---

## Usage

**Script:**
```bash
python rocket_landing_mpc_lqr.py
```

**Notebook:**
```bash
jupyter notebook rocket_landing.ipynb
```

> MPC simulation takes 30–60 seconds on a standard laptop. It solves a 30-variable constrained QP at each of ~270 timesteps.

---

## Tuning Guide

The only parameters you need to change to modify controller behaviour:

**Q matrix** — state penalties. Increase `Q[4,4]` (theta) to make the controller more aggressive about keeping the rocket upright. Increase `Q[3,3]` (vz) to prioritise descent rate control.

**R matrix** — control penalties. Increase `R[1,1]` (delta) to reduce gimbal deflection — useful if the nozzle is saturating. Decrease it to allow more aggressive attitude correction.

**N** — MPC prediction horizon. Longer horizon = better foresight, softer landing, slower computation. Shorter = faster but more myopic. Minimum useful value is approximately 8 steps (0.8s).

**dt** — control frequency. Smaller dt = more accurate but more computation. The linearisation validity improves with smaller dt.

> ⚠️ **Common mistake**: setting R too small (e.g., `diag([0.1, 0.1])`) causes the controller to command gimbal angles far outside ±20°. Clamping destroys the stability guarantee and the rocket diverges immediately. Always verify that the unclamped LQR commands stay within physical limits.

---

## Theory References

| Topic | Reference |
|---|---|
| Discrete LQR and DARE | Åström & Wittenmark — *Computer-Controlled Systems* |
| Condensed MPC formulation | Maciejowski — *Predictive Control with Constraints* |
| Rocket landing guidance | Acikmese & Ploen — *Convex Programming Approach to Powered Descent Guidance* (2007) |
| SLSQP algorithm | Kraft — *A Software Package for Sequential Quadratic Programming* (1988) |
| State space control foundations | Franklin, Powell & Emami-Naeini — *Feedback Control of Dynamic Systems* |

---

## Extensions

- **3D simulation** — add y-axis, roll and yaw. State grows to 12, two gimbal axes as inputs. MPC formulation scales directly.
- **Wind disturbance** — add stochastic horizontal force to ODE. MPC re-plans from disturbed state automatically; LQR does not.
- **Kalman filter** — add sensor noise, design discrete KF, feed estimated state instead of true state.
- **Nonlinear MPC** — replace linearised prediction with full ODE inside the QP using CasADi + IPOPT.
- **Monte Carlo** — run 100+ trials with randomised initial conditions, compare success rates.
- **Mass depletion** — reduce m over time (Tsiolkovsky), requires gain scheduling or re-linearisation.
- **Trajectory optimisation** — compute fuel-optimal descent path first (lossless convexification), track with MPC.

---

## License

MIT License — see `LICENSE` for details.

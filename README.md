# Water Bath Temperature Control System

**College Engineering Project — Part A: Mathematical Modelling, Simulation, and Control**

This repository contains the Python-based engineering simulation and closed-loop control system for a temperature-regulated water bath. It serves as the theoretical and numerical foundation (Part A) preceding physical microcontroller implementation (Part B).

---

## 1. Project Overview & Requirements

- **Application**: Temperature-controlled laboratory/industrial water bath.
- **Operating Range**: 30°C to 90°C.
- **Control Resolution**: 1°C.
- **Hardware Prototype Range (Part B context)**: 30°C to 60°C.
- **Stack**: Python 3.14, NumPy, SciPy (`solve_ivp`), Matplotlib, Pandas, Pytest, managed with `uv`.

---

## 2. Mathematical Foundation: Lumped-Capacitance Thermal Model

The system is modelled using the first-order lumped-capacitance energy balance:

$$m \cdot c_p \frac{dT}{dt} = P_{\text{heater}}(t) - UA (T(t) - T_{\text{ambient}})$$

### Physical Parameters

| Parameter | Symbol | SI Unit | Typical Baseline | Physical Significance |
| :--- | :--- | :--- | :--- | :--- |
| **Water Mass** | $m$ | $\text{kg}$ | $1.0\text{ to }2.0\text{ kg}$ | Fluid mass governing thermal inertia ($m = \rho \cdot V$). |
| **Specific Heat Capacity** | $c_p$ | $\text{J}/(\text{kg}\cdot\text{K})$ | $\approx 4184\text{ J}/(\text{kg}\cdot\text{K})$ | Energy required to raise $1\text{ kg}$ of water by $1\text{ K}$. |
| **Thermal Capacitance** | $C_{\text{th}} = m c_p$ | $\text{J}/\text{K}$ | $4184\text{ J}/\text{K}$ (at $1\text{ kg}$) | Resistance to rate of temperature change. |
| **Overall Heat Loss Coefficient** | $UA$ | $\text{W}/\text{K}$ | $2.0\text{ to }10.0\text{ W}/\text{K}$ | Lumped convective/conductive/radiative heat loss rate. |
| **Thermal Resistance** | $R_{\text{th}} = \frac{1}{UA}$| $\text{K}/\text{W}$ | Inverse of $UA$ | Thermal resistance between water and ambient air. |
| **Thermal Time Constant** | $\tau = \frac{m c_p}{UA}$ | $\text{s}$ | $500\text{ to }2000\text{ s}$ | Characteristic response time of the first-order system. |
| **Ambient Temperature** | $T_{\text{ambient}}$ | $\text{K}$ (or $^\circ\text{C}$) | $20\text{ to }25^\circ\text{C}$ | Temperature of surrounding environment. |
| **Heater Power** | $P_{\text{heater}}$ | $\text{W}$ | $300\text{ to }1000\text{ W}$ | Thermal heat input rate delivered by the heating element. |

### Analytical Solutions

Under constant heater power $P_{\text{heater}}$ and constant ambient temperature $T_{\text{ambient}}$:

- **Equilibrium (Steady-State) Temperature**:
  $$T_{\text{equilibrium}} = T_{\text{ambient}} + \frac{P_{\text{heater}}}{UA}$$

- **Transient Temperature Trajectory**:
  $$T(t) = T_{\text{equilibrium}} + (T_0 - T_{\text{equilibrium}}) \cdot e^{-t / \tau}$$

### Engineering Assumptions & Limitations
1. **Lumped-Capacitance Validity**: Assumes uniform water temperature ($\nabla T \approx 0$). In physical systems, this requires adequate convective mixing or a circulation stirrer ($\text{Bi} = \frac{h L_c}{k} < 0.1$).
2. **Container Thermal Mass**: The thermal capacitance of the vessel walls and heater element casing is initially neglected.
3. **Evaporation Losses**: Surface mass transfer and evaporative latent heat loss are neglected at lower temperatures, but become significant above 60°C.
4. **Constant Properties**: Fluid density $\rho$, specific heat $c_p$, and heat loss coefficient $UA$ are assumed constant over the operating range.

---

## 3. Project Architecture

The project follows a modular structure separating physical models, actuator dynamics, controllers, and evaluation metrics:

```text
ASI/
├── pyproject.toml               # Project metadata & dependencies
├── README.md                    # Project documentation & engineering plan
├── src/
│   └── water_bath/
│       ├── __init__.py
│       ├── config.py            # Physical constants & system parameters dataclasses
│       ├── thermal_model.py     # Lumped thermal ODE & analytical solutions
│       ├── heater.py            # Actuator saturation, limits & duty-cycle switching
│       ├── controllers/
│       │   ├── __init__.py
│       │   ├── on_off.py        # Hysteresis-based bang-bang controller
│       │   └── pid.py           # Discrete PID with anti-windup & derivative filter
│       └── metrics.py           # Rise time, settling time, overshoot, IAE, ISE
├── tests/                       # Automated test suite
│   ├── conftest.py
│   ├── test_thermal_model.py
│   ├── test_heater.py
│   └── test_controllers.py
├── scripts/                     # Standalone simulation & demonstration scripts
│   ├── simulate_open_loop.py
│   ├── simulate_on_off.py
│   └── simulate_pid.py
└── reports/                     # Simulation results, plots, and CSV tabular data
    ├── plots/
    └── data/
```

---

## 4. Staged Development Workflow

- [x] **Stage 0: Repository Inspection and Project Planning** (Current)
  - Inspect repository structure, Python runtime, Git status, and toolchain.
  - Define physical equations, unit conventions, and project architecture.
- [ ] **Stage 1: Open-Loop Thermal Modelling**
  - Implement continuous-time thermal ODE using `scipy.integrate.solve_ivp`.
  - Simulate heating and passive cooling trajectories.
  - Verify numerical simulation against closed-form analytical solutions.
- [ ] **Stage 2: Model Validation and Testing**
  - Implement comprehensive Pytest suite for energy conservation, equilibrium verification, and parameter sensitivity.
- [ ] **Stage 3: Heater Modelling**
  - Implement separate actuator model with maximum power rating, saturation, and duty-cycle control.
- [ ] **Stage 4: ON/OFF Temperature Controller**
  - Implement hysteresis-based bang-bang controller and analyse steady-state limit cycle oscillations.
- [ ] **Stage 5: PID Controller**
  - Implement discrete-time PID controller with integral anti-windup clamping and derivative low-pass filtering.
- [ ] **Stage 6: PID Tuning and Performance Analysis**
  - Systematic tuning (e.g., Cohen-Coon / Ziegler-Nichols step response method) and transient metric quantification (rise time, settling time, overshoot, IAE, ISE).
- [ ] **Stage 7: Controller Comparison and Disturbance Testing**
  - Benchmark ON/OFF vs PID under setpoint changes, ambient shifts, and thermal shock disturbances.
- [ ] **Stage 8: Final Results and College Project Documentation**
  - Produce publication-ready comparison charts, CSV export summaries, and hardware transition guide for Part B.

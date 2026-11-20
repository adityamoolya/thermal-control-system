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

## 2. Engineering Concepts & Physical Principles

### The Lumped-Capacitance Assumption
We model the water bath using the first-order **lumped-capacitance method**:

$$m \cdot c_p \frac{dT}{dt} = P_{\text{heater}}(t) - UA (T(t) - T_{\text{ambient}})$$

#### 1. Why "Lumped Capacitance"?
In classical heat transfer, thermal conduction within a body and convection at its boundaries occur simultaneously. A distributed model would require solving the partial differential equation (PDE) for transient conduction:
$$\rho c_p \frac{\partial T}{\partial t} = k \nabla^2 T$$

The **lumped-capacitance approximation** simplifies this by treating the entire liquid volume as having a **spatially uniform temperature** $T(t)$ at any instant in time, reducing the system to an ordinary differential equation (ODE).

*Physical Validity and Justification*:
- This assumption is valid if internal thermal conduction/mixing is orders of magnitude faster than heat loss to the ambient environment. This is governed by the dimensionless **Biot number**:
  $$\text{Bi} = \frac{h L_c}{k} < 0.1$$
  Where $h$ is the convective heat transfer coefficient, $L_c = \frac{V}{A_{\text{surface}}}$ is the characteristic length, and $k$ is thermal conductivity.
- In stagnant water, natural convection alone can create vertical temperature stratification (warmer water rising to the top). In a practical physical water bath, **continuous mechanical stirring or water circulation** is employed to enforce spatial uniformity, ensuring the lumped-capacitance model closely mirrors reality.

---

### Physical Meaning of System Parameters

All calculations are maintained strictly in **SI units**:

| Parameter | Symbol | SI Unit | Typical Baseline | Physical Significance |
| :--- | :--- | :--- | :--- | :--- |
| **Water Mass** | $m$ | $\text{kg}$ | $1.0\text{ to }2.0\text{ kg}$ | Fluid mass governing thermal inertia ($m = \rho \cdot V$). |
| **Specific Heat Capacity** | $c_p$ | $\text{J}/(\text{kg}\cdot\text{K})$ | $\approx 4184\text{ J}/(\text{kg}\cdot\text{K})$ | Energy required to raise $1\text{ kg}$ of water by $1\text{ K}$. |
| **Thermal Capacitance** | $C_{\text{th}} = m c_p$ | $\text{J}/\text{K}$ | $4184\text{ J}/\text{K}$ (at $1\text{ kg}$) | Overall capacity of the liquid body to store thermal energy. |
| **Overall Heat Loss Coefficient** | $UA$ | $\text{W}/\text{K}$ | $2.0\text{ to }10.0\text{ W}/\text{K}$ | Lumped rate of conductive, convective, and radiative heat loss per Kelvin difference to ambient. |
| **Thermal Resistance** | $R_{\text{th}} = \frac{1}{UA}$| $\text{K}/\text{W}$ | Inverse of $UA$ | Resistance opposing heat flow from the bath to the surroundings. |
| **Thermal Time Constant** | $\tau = R_{\text{th}} C_{\text{th}} = \frac{m c_p}{UA}$ | $\text{s}$ | $500\text{ to }2000\text{ s}$ | Characteristic time for the system to respond to a step change in power. |
| **Ambient Temperature** | $T_{\text{ambient}}$ | $\text{K}$ (or $^\circ\text{C}$) | $20\text{ to }25^\circ\text{C}$ | Boundary temperature of the surrounding air/room. |
| **Heater Power** | $P_{\text{heater}}$ | $\text{W}$ | $300\text{ to }1000\text{ W}$ | Thermal heat input rate delivered into the liquid by the electric element. |

---

### Mathematical Derivation: Equilibrium & Transient Response

Rearranging the governing first-order linear differential equation into standard dynamic form:
$$\frac{dT}{dt} + \frac{UA}{m c_p} T(t) = \frac{UA}{m c_p} T_{\text{ambient}} + \frac{P_{\text{heater}}}{m c_p}$$

Substituting the thermal time constant $\tau = \frac{m c_p}{UA}$:
$$\frac{dT}{dt} + \frac{1}{\tau} T(t) = \frac{1}{\tau} \left( T_{\text{ambient}} + \frac{P_{\text{heater}}}{UA} \right)$$

#### 1. Steady-State Equilibrium Temperature ($T_{\text{equilibrium}}$)
At thermal equilibrium, the temperature ceases to change ($\frac{dT}{dt} = 0$). The rate of electrical heat supplied equals the rate of heat lost to the environment:
$$P_{\text{heater}} = UA (T_{\text{equilibrium}} - T_{\text{ambient}})$$
$$T_{\text{equilibrium}} = T_{\text{ambient}} + \frac{P_{\text{heater}}}{UA}$$

#### 2. Exact Analytical Transient Solution
Using the integrating factor $\mu(t) = e^{t/\tau}$, the closed-form exact solution for constant $P_{\text{heater}}$ and $T_{\text{ambient}}$ with initial temperature $T(0) = T_0$ is:
$$T(t) = T_{\text{equilibrium}} + (T_0 - T_{\text{equilibrium}}) \cdot e^{-t / \tau}$$

**Significance for Verification**:
Because this exact closed-form analytical equation exists for constant inputs, we can benchmark numerical solvers (`scipy.integrate.solve_ivp`) directly against analytical truth to verify integration tolerance and energy conservation before introducing complex non-linear controllers.

---

### Engineering Assumptions & Model Limitations
1. **Lumped Capacitance**: Assumes zero spatial temperature gradients ($\nabla T \approx 0$). Physically requires a circulation pump or stirrer.
2. **Container Thermal Mass**: The thermal mass of the container walls, insulation, and the heating element jacket is neglected in this initial model.
3. **Evaporative Cooling**: At temperatures approaching 60°C–90°C, surface evaporation becomes an increasingly significant non-linear heat loss mechanism ($q_{\text{evap}} = \dot{m}_{\text{evap}} \cdot h_{fg}$). This is initially omitted in the linear $UA$ model and will be evaluated during disturbance analysis.
4. **Constant Thermophysical Properties**: Density ($\rho$), specific heat capacity ($c_p$), and heat transfer coefficient ($UA$) are treated as constant over the operating range.
5. **Actuator Idealisation**: In the initial model, heater power is assumed to be instantaneously delivered without internal thermal lag of the heating coil.

---

## 3. Project Architecture

The codebase follows a modular structure separating physical models, actuator dynamics, controllers, simulation runners, and performance metrics:

```text
ASI/
├── pyproject.toml               # Project metadata & dependencies
├── README.md                    # Project documentation & engineering explanations
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

- [x] **Stage 0: Repository Inspection and Project Planning** (Completed)
  - Inspected repository structure, toolchain, Python 3.14 runtime, and Git environment.
  - Documented physical equations, unit conventions, analytical derivations, and system architecture.
- [ ] **Stage 1: Open-Loop Thermal Modelling**
  - Implement continuous-time thermal ODE using `scipy.integrate.solve_ivp`.
  - Simulate open-loop heating and passive cooling trajectories.
  - Compare numerical results against closed-form analytical solutions to quantify solver residual error.
- [ ] **Stage 2: Model Validation and Testing**
  - Pytest automated test suite for energy conservation, equilibrium verification, and parameter sensitivity.
- [ ] **Stage 3: Heater Modelling**
  - Actuator sub-model with power saturation ($0 \le P \le P_{\max}$), quantization, and switching logic.
- [ ] **Stage 4: ON/OFF Temperature Controller**
  - Hysteresis-based bang-bang controller and analysis of limit-cycle switching oscillations.
- [ ] **Stage 5: PID Controller**
  - Discrete-time PID controller with anti-windup clamping and low-pass derivative filtering.
- [ ] **Stage 6: PID Tuning and Performance Analysis**
  - Systematic tuning methods and transient performance metrics (rise time, settling time, overshoot, IAE, ISE).
- [ ] **Stage 7: Controller Comparison and Disturbance Testing**
  - Side-by-side benchmark of ON/OFF vs PID under setpoint transitions, ambient fluctuations, and cold-water disturbances.
- [ ] **Stage 8: Final Results and College Project Documentation**
  - Production of publication-quality comparative plots, CSV data exports, and preparation for Part B microcontroller implementation.

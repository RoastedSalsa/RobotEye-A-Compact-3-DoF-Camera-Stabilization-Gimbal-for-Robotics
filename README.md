# RobotEye — A Compact 3-DoF Camera Stabilization Gimbal for Robotics

> **BSc Final Project (Bachelor Eindopdracht)** · TU Eindhoven — Mechanical Engineering, Robotics Section · BEP No. RBTBEP25020 · Graded **9/10**
> Supervisors: Jordy Senden, René van de Molengraft · September 2025

A compact **3-DoF camera stabilization gimbal** built around a hybrid architecture: a **2-DoF spherical parallel mechanism** for pitch and roll, paired with a **direct-drive yaw axis** at the end-effector. Designed to give legged robots, drones, and robot arms a stable visual platform without the bulk and inertia of a conventional 3-axis serial gimbal.

The system performs real-time IMU-based stabilization using quaternion math and integrates an OpenCV color-tracking pipeline running on a Raspberry Pi 4.

<p align="center">
  <img src="docs/demo.gif" alt="RobotEye stabilization and tracking demo" width="500"/>
</p>

<!-- TODO: extract a 3–6 second clip from the bandicam videos (stabilization + tracking) and save as docs/demo.gif -->

---

## Why this architecture

Most commercial mini-gimbals use a serial 3-motor configuration. They work, but they're **bulky** (~10 cm height, >250 g), and the stacked motor arrangement places mass far from the rotation axes — driving up moments of inertia and limiting bandwidth. For small robots like the Mini-Cheetah or aerial platforms, that's a problem.

RobotEye explores a hybrid alternative:

- A **2-DoF spherical parallel mechanism** places both base motors **on the structure's base**, so the end-effector inertia stays low and stiffness stays high.
- A **direct-drive yaw motor** sits at the camera end-effector — the camera mass is tiny (3 g for a Pi Camera V2), so the inertia penalty is negligible.
- The result avoids the **backlash of geared designs**, the **belt tensioning headaches** of cable-driven systems, and the **inertia stacking** of pure direct-drive gimbals — at the cost of more complex (coupled) inverse kinematics.

The trade-off is deliberate: more math, less mechanical compromise.

---

## System specs

| Subsystem | Component | Notes |
|---|---|---|
| Actuators | 3× iPower **GM2804** BLDC gimbal motors | Selected over the ideal Faulhaber 1509B due to EU availability |
| Motor control | **SimpleFOC** with FOC | Cascaded position + velocity PID per motor |
| Drivers | 3× SparkFun **TMC6300** breakouts | One per motor |
| Encoders | 3× **AS5600** magnetic absolute encoders | 12-bit, I²C, on-axis with diametric magnets |
| IMU | Bosch **BNO055** 9-DoF | Quaternion output via I²C, frame-mounted |
| MCUs | **Teensy 4.1** (base motors) + **Teensy 4.0** (yaw motor) | UART link between them; 18 PWM pairs total |
| SBC (vision) | **Raspberry Pi 4** | Runs OpenCV pipeline, sends pixel offset to Teensy 4.1 via UART |
| Camera | Waveshare **IMX219** (8 MP, 120° FoV) | Mounted on end-effector |
| Power | 11.1 V 3S LiPo + buck converters (5 V / 10 V) | Battery is for standalone demo; production would tap robot's bus |
| Frame | 3D-printed **PLA** with heat-set threaded inserts | Iterative redesign from friction-fit prototype |
| Language | **C++** (firmware) + **Python/OpenCV** (vision) | |

---

## Control architecture

```
            ┌──────────────────────────┐
            │  Raspberry Pi 4 (vision) │
            │  OpenCV HSV tracker      │──── pixel offset (UART)
            └──────────────────────────┘                      │
                                                              ▼
  ┌─────────────┐    quaternion     ┌──────────────────────────────┐
  │   BNO055    │ ────────────────► │   Teensy 4.1                 │
  │   IMU       │                   │                              │
  └─────────────┘                   │   q_rel = q_init* ⊗ q_curr   │
                                    │   → Euler error              │
                                    │   + cumulative CV error      │
                                    │   → analytical inverse       │
                                    │      kinematics (SPM)        │
                                    │   → cascaded position +      │
                                    │      velocity PID (SimpleFOC)│
                                    └──────────────────────────────┘
                                              │           ▲
                                       PWM    ▼           │ encoder feedback
                                    ┌─────────────┐    ┌──────────┐
                                    │ TMC6300 ×3  │───►│ GM2804×3 │
                                    └─────────────┘    └──────────┘
```

**Key design choices:**

- **Analytical inverse kinematics**, adapted from Mghames et al. (2019), running on the MCU. A numerical Python solver was prototyped first to validate the geometry but was too slow to deploy on hardware.
- **Quaternion-based orientation** error: the relative orientation is computed as `q_rel = q_init* ⊗ q_curr` (Hamilton product with the initial-pose conjugate), then converted to Euler angles for the IK solver.
- **Singularity avoidance** by limiting motor 2 to `−π/2 < q₂ < π/2`.
- **Color-based object tracking** in OpenCV: HSV mask → contour detection → bounding box → pixel offset converted to angle via `(pixel_offset / frame_length) · FoV`.

---

## Results

The project set five quantitative objectives. Honest scorecard:

| Objective | Target | Achieved | Met? |
|---|---|---|---|
| Disturbance rejection | ≤ 10 Hz @ ≤ 10° amp, error ≤ 3° | Performance degrades above ~3 Hz | ✗ |
| Stabilization error | ≤ 0.05 rad | 0.02 rad at center → 0.1 rad at 0.8 rad displacement | Partial |
| Tracking range | ≥ 70° H & V | ±80° pitch, ±120° roll & yaw | ✓ |
| Tracking settling time | ≤ 1 s | ~2 s | ✗ |
| Tracking static error | ≤ 3° | within 3° | ✓ |
| Envelope | ≤ 100 × 100 × 100 mm | 100 × 70 × 100 mm | ✓ |
| Material cost | ≤ €300 (target €200) | ~€300 | Met cap |
| Camera support | Pi Camera V2 / IMX219 | Yes | ✓ |

**Why the bandwidth fell short:** the limiting factors were (a) the GM2804 motors, chosen for EU availability rather than ideal performance — the smaller, lighter Faulhaber 1509B would have ~4× less mass and proportionally lower inertia, and (b) BNO055 sensor lag and PID tuning constraints. The 9/10 grade reflects that the **architecture was successfully validated** even where the prototype's specific component choices limited absolute performance.

The thesis includes a detailed future-improvements section outlining the path to closing the gap: smaller motors, a custom STM32-based control board, custom PCBs replacing breadboards, a higher-grade IMU, dynamic-model-based feedforward control, and a programmable turntable test rig for repeatable bandwidth measurement.

---

## Repository structure

```
.
├── firmware/         # Teensy 4.1 + 4.0 C++ control code (SimpleFOC)
├── vision/           # Raspberry Pi OpenCV tracking pipeline
├── kinematics/       # Python prototype of analytical & numerical IK
├── cad/              # Onshape exports of frame, links, joints
├── docs/
│   ├── thesis.pdf    # Full BEP report
│   ├── demo.gif      # Stabilization + tracking demo
│   └── figures/      # Diagrams, photos, plots
└── README.md
```

<!-- TODO: adjust to match the actual repo layout once organized -->

---

## Documents

- [**Full thesis report (PDF)**](docs/thesis.pdf) — design rationale, kinematics derivation, full results
- Demo videos — stabilization and object tracking
- [**Final presentation slides**](docs/Final_BEP_presentation.pdf)

---

## Continuation

This project is being continued as a research repo investigating the dynamics, control, and design refinement of the spherical parallel mechanism:
**[2-DoF-Spherical-Parallel-Manipulator](https://github.com/RoastedSalsa/2-DoF-Spherical-Parallel-Manipulator)**

---

## Author

**Augustas Gerardas Pugžlys**
BSc Mechanical Engineering (Cum Laude), TU Eindhoven
MSc Robotics, TU Delft (2026 – )
[GitHub](https://github.com/RoastedSalsa) · [LinkedIn](#) <!-- TODO: add LinkedIn URL -->

---

## License

Released under the MIT License. See [`LICENSE`](LICENSE) for details.

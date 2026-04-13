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


## Documents

- [**Full thesis report (PDF)**](docs/thesis.pdf) — design rationale, kinematics derivation, full results

---

## Continuation

This project is being continued as a research repo investigating the dynamics, control, and design refinement of the spherical parallel mechanism:
**[2-DoF-Spherical-Parallel-Manipulator](https://github.com/RoastedSalsa/2-DoF-Spherical-Parallel-Manipulator)**

---

## Author

**Augustas Gerardas Pugžlys**
BSc Mechanical Engineering (Cum Laude), TU Eindhoven
MSc Robotics, TU Delft (2026 – )
[GitHub](https://github.com/RoastedSalsa) · [LinkedIn](https://www.linkedin.com/in/augustas-gerardas-pugzlys/) <!-- TODO: add LinkedIn URL -->

---

## License

Released under the MIT License. See [`LICENSE`](LICENSE) for details.

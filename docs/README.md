# docs/

| Document | Use it for |
|---|---|
| [PLAN.md](PLAN.md) | Prep plan, status tracker, open questions, collected talking points |
| [REFERENCE.md](REFERENCE.md) | **Main cheat sheet:** EKF matrices and equations, NEES/NIS + χ² tables, Q from specs, ⊞/⊟, terrain registration, frames, lunar facts, `rclpy` cheat sheet, design and presentation checklists |
| [REFERENCE_3D.md](REFERENCE_3D.md) | Planar → 3D error-state EKF: SO(3) toolkit, IMU kinematics, 15×15 Jacobian, 3D measurement Jacobians, observability, ROS 3D gotchas |
| [ADDING_SENSORS.md](ADDING_SENSORS.md) | Recipe + tested examples (wheel speed, position fix, landmark range/bearing, sun sensor, DEM pose); relative, delayed and degenerate cases |
| [ARCHITECTURE.md](ARCHITECTURE.md) | How this repo's code is structured: types, Protocols, components, data flow, extension points, limitations |
| [ROS2_INTEGRATION.md](ROS2_INTEGRATION.md) | Mapping this EKF onto ROS 2 nodes, what exists in the local rover workspace, practice plan, pitfalls |
| [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md) | Strawman lunar-rover localization design: clarifying questions, architecture, sensor roles, decisions, fault handling, validation |
| [ONSITE_LOG.md](ONSITE_LOG.md) | Day-of running log: task, assumptions, timestamped decisions/results, presentation material |
| [PRESENTATION_TEMPLATE.md](PRESENTATION_TEMPLATE.md) | Fill-in outline for the end-of-day talk (what / why / worked / didn't / next), timing, likely questions |
| [../ros2_template/README.md](../ros2_template/README.md) | Known-good `rclpy` EKF package (node, fake star tracker, params, launch) — copy and adapt |

Agent context lives at the repo root: `AGENTS.md` (and `CLAUDE.md`, which imports it).

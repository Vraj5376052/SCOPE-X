# SCOPE-X — Smart Continuous Optical Patrol Engine

**3003ICT Programming for Robotics — Final Group Project**
**Group NA9:** Vraj Patel, Vo Duc Anh Tran, Manveer Singh
**Specialisation:** Option B — Autonomous Robotics (Webots)

SCOPE-X is an autonomous security patrol robot built on the Webots E-puck platform. It patrols a multi-room arena, detects intruders inside a restricted zone using a camera, classifies threats based on spatial context, and intercepts confirmed threats while maintaining safe operation at all times.

---

## Quick Start

1. Open Webots R2025a
2. `File → Open World…` → `worlds/scope_world.wbt`
3. Press play — the robot begins patrolling automatically
4. Console output shows state transitions and threat levels in real time

---

## Project Structure

```
SCOPE-X/
├── worlds/
│   └── scope_world.wbt          # 3x3 m arena, 4 rooms, restricted zone, intruder
└── controllers/
    └── scope_controller/
        └── scope_controller.py  # Robot brain (Sense → Think → Act FSM)
```

---

## How It Works

The robot runs a **Sense → Think → Act** loop every simulation tick.

**Sense** — reads 8 IR proximity sensors (with moving-average filtering), a 64×48 RGB camera, GPS, and an InertialUnit for heading.

**Think** — a prioritised FSM evaluates sensor data and transitions between 10 states:

| Priority | Condition | State entered |
|---|---|---|
| 1 | Front IR > 1800 | `EMERGENCY_STOP` |
| 2 | Camera detects threat (MEDIUM/HIGH) | `INTRUDER_DETECTED` |
| 3 | Front IR > 300, no threat | `AVOID_OBSTACLE` |
| 4 | Default | `PATROL` |

**Act** — drives differential motors, sets LED patterns, and triggers TTS announcements.

---

## FSM States

| State | What it does |
|---|---|
| `PATROL` | Follows a lawnmower waypoint path through all rooms using GPS + InertialUnit heading control |
| `SCAN` | Rotates 360° to search for or reacquire the intruder |
| `INTRUDER_DETECTED` | Stops and confirms threat for 600 ms before escalating |
| `NAVIGATE_TO_TARGET` | GPS-guided route to the RZ1 doorway, then switches to camera centroid servoing inside |
| `INTERCEPT` | Holds a safe standoff distance from the intruder |
| `ALERT` | Stops, blinks LEDs, and speaks a context-specific warning |
| `AVOID_OBSTACLE` | 3-phase side-step: turn 90°, slide forward, realign to original heading |
| `EMERGENCY_STOP` | Full stop on imminent collision |
| `RECOVERY` | Reverses then rotates to escape a stall |
| `NO_ENTRY_ZONE` | Halts at the BL room threshold, announces no-entry, skips BL waypoints |

---

## Threat Classification

The robot classifies threats using both camera and GPS together — the same red object gets a different threat level depending on where the robot is:

- **NONE** — no red pixels detected
- **LOW** — red visible, robot far from the restricted zone (monitor only)
- **MEDIUM** — red visible, robot near the RZ boundary
- **HIGH** — red visible, robot inside the restricted zone

This drives the entire mission chain: LOW is ignored during patrol, MEDIUM/HIGH trigger interception.

---

## Arena Layout

- 4 rooms separated by walls with doorway gaps
- **TR room (RZ1)** — Server room, marked with an orange floor. Restricted zone protected by the robot.
- **BL room** — No-entry zone for the robot itself, marked with a red X on the floor. Robot halts at the threshold and announces before turning back.
- Central corridor connecting top and bottom halves

---

## Safety Features

- **EMERGENCY_STOP** triggers when front IR exceeds 1800 (confirmed by flank sensors)
- **RECOVERY** sequence (reverse 0.8 s → rotate 1.2 s) escapes stalls automatically
- **Stall watchdog** detects if the robot is driving against an obstacle and forces RECOVERY
- Intruder detection takes priority over obstacle avoidance so the robot doesn't swerve around the red box it should be intercepting

---

## Submission Checklist

- [x] Source code — this repository
- [ ] 4-page report
- [ ] 5-minute video demo

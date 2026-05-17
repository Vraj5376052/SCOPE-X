# SCOPE-X — Smart Continuous Optical Patrol Engine

**3003ICT Programming for Robotics — Final Group Project**
**Group NA9:** Vraj Patel, Vo Duc Anh Tran, Manveer Singh
**Specialisation:** Option B — Autonomous Robotics (Webots)

SCOPE-X is an autonomous security patrol robot built on the Webots E-puck platform. It patrols a multi-room arena, detects red intruders inside restricted zones using a camera, classifies threats using context-aware perception, and intercepts confirmed threats — while always prioritising collision safety.

---

## Quick start

1. Open Webots R2025a.
2. `File → Open World…` → `worlds/scope_world.wbt`.
3. Press the play button. The robot begins patrolling automatically.
4. Console output prints state transitions and threat classifications.

---

## Project structure

```
scope_claude/
├── README.md                 # this file
├── worlds/
│   └── scope_world.wbt       # 5×5 m arena, 4 rooms, 2 restricted zones, intruder
└── controllers/
    └── scope_controller/
        └── scope_controller.py   # SCOPE-X brain (Sense → Think → Act FSM)
```

This matches the Wk05 lecture file layout (`worlds/`, `controllers/<name>/<name>.py`).

---

## How the project satisfies every Technical Requirement

The project spec (`3003ICT-Project.pdf §4`) lists Core Requirements that apply to every submission, plus the Option B specialisation. Every requirement is mapped to its exact implementation below.

### Core Requirements

| Requirement | How SCOPE-X satisfies it |
| --- | --- |
| **≥ 2 inputs (sensors / perception sources)** | (1) 8 × IR proximity sensors `ps0`–`ps7`, (2) RGB camera. Both enabled with `enable(TIME_STEP)` per Wk05 init pattern. |
| **≥ 2 outputs (actuators / behaviours)** | (1) Differential-drive motors (`left wheel motor`, `right wheel motor`), (2) Ring of 10 LEDs (`led0`–`led9`) for solid/blinking alert patterns. |
| **FSM or Behaviour Tree** | Explicit FSM (`class FSM:`), single `state` variable, transitions evaluated each tick — exactly the Wk04 `switch-case` pattern. |
| **≥ 4 behavioural states** | **9 states**: `PATROL`, `SCAN`, `INTRUDER_DETECTED`, `NAVIGATE_TO_TARGET`, `INTERCEPT`, `ALERT`, `AVOID_OBSTACLE`, `EMERGENCY_STOP`, `RECOVERY`. |
| **Multi-condition decision logic** | Threat classification fuses **two independent signals** — `red_ratio` (object) AND `zone_ratio` (spatial context) — to emit `LOW / MEDIUM / HIGH`. Transitions like `if threat in ("MEDIUM","HIGH") and state in (PATROL, SCAN)` use multi-variable predicates. |
| **Safety / fail-safe mechanism** | Three-tier safety: (1) `EMERGENCY_STOP` on imminent collision, (2) `RECOVERY` (reverse-then-rotate) on stalls, (3) hard prioritisation that prevents mission states from overriding obstacle avoidance. |
| **Structured system architecture diagram** | See ASCII diagram below + `## System architecture` section. |

### Option B — Autonomous Robotics (Webots)

| Spec requirement | How SCOPE-X satisfies it |
| --- | --- |
| **Navigation** | Autonomous patrol pattern (forward / gentle-turn cycle) explores all four rooms. Visual servoing in `NAVIGATE_TO_TARGET` steers toward the centroid of red pixels. |
| **Obstacle avoidance** | Reactive turn-away in `AVOID_OBSTACLE`, biased by which side has the strongest reading. `OBSTACLE_THRESHOLD = 80` matches the Wk05 lecture value. |
| **Perception-driven decision** | Camera RGB sampling drives the entire mission layer. `PATROL → INTRUDER_DETECTED → NAVIGATE → INTERCEPT → ALERT` is gated entirely by what the camera sees. |

### Advanced / Contextual Component (10 %)

The pitch nominates **Context-Aware Threat Classification** as the advanced component, which counts under both *AI / Adaptive Behaviour* and *Perception (Vision-Based)* in the marking rubric. Implementation:

- Same red object → different threat level depending on whether the camera also sees a restricted-zone floor.
- HIGH = large red blob *and* zone visible (close-range intruder inside RZ).
- MEDIUM = blob *or* zone visible (entering RZ, or red object distant but classified).
- LOW = blob only, far away, no zone context (monitor only).
- This is true sensor fusion + classification (Wk04 "Sensor Fusion Concept" and "Classification (Contextual Intelligence)").

---

## System architecture (Sense → Think → Act)

```
┌────────────────────────────────────────────────────────────┐
│                      EXTERNAL WORLD                        │
│             (arena, walls, RZ floors, intruder)            │
└─────────────────────────────┬──────────────────────────────┘
                              │
        ┌─────────────────────┴────────────────────┐
        │            SENSE  (Layer 1)              │
        │  • 8 × IR proximity (ps0..ps7)           │
        │  • RGB camera (64×48, FOV 1.0 rad)       │
        │  • Moving-average filter on IR (Wk04)    │
        └─────────────────────┬────────────────────┘
                              │
        ┌─────────────────────┴────────────────────┐
        │           THINK  (Layer 2 - FSM)         │
        │                                          │
        │  Priority 1  EMERGENCY_STOP  (safety)    │
        │  Priority 2  AVOID_OBSTACLE  (reflex)    │
        │  Priority 3  Mission FSM:                │
        │     PATROL → SCAN → INTRUDER_DETECTED    │
        │       → NAVIGATE_TO_TARGET → INTERCEPT   │
        │       → ALERT  (back to PATROL when      │
        │       threat clears)                     │
        │  Priority 4  RECOVERY (stall escape)     │
        │                                          │
        │  Threat classifier (Layer 3 - "Strategy"):│
        │     red_ratio + zone_ratio  →  LOW /     │
        │       MEDIUM / HIGH                      │
        └─────────────────────┬────────────────────┘
                              │
        ┌─────────────────────┴────────────────────┐
        │             ACT  (Layer 1)               │
        │  • Differential drive (set_velocity)     │
        │  • LED ring (solid / blink alert)        │
        │  • Console state log (audit trail)       │
        └──────────────────────────────────────────┘
```

This three-layer split (Hardware → Reflex/FSM → Strategy/AI) is the architecture pattern from Wk05 ("Layered Behavior Architecture") and Wk04 ("3-Layer Brain").

---

## FSM state table

| State | Trigger to enter | Behaviour | Exit condition |
| --- | --- | --- | --- |
| `PATROL` | Default / threat cleared | Forward cycle with periodic gentle turn | Obstacle, threat detected |
| `SCAN` | Lost sight of target while navigating | Rotate in place looking for red | Red re-acquired, or 3 s timeout → `PATROL` |
| `INTRUDER_DETECTED` | `threat ∈ {MEDIUM, HIGH}` while in `PATROL`/`SCAN` | Stop, LEDs solid, classify | After 600 ms → `NAVIGATE_TO_TARGET` (or back to `PATROL` if threat dropped) |
| `NAVIGATE_TO_TARGET` | After classification | Visual-servo toward red centroid; LEDs blink | Red close (`red_ratio ≥ 0.10`) or front sensor close → `INTERCEPT`. Lost sight → `SCAN`. |
| `INTERCEPT` | Close to target | Hold safe stand-off (back off / creep / hold) | After 1.5 s → `ALERT` |
| `ALERT` | Confirmed threat held | Stationary, blinking LEDs (sustained warning) | `threat == NONE` → `PATROL` |
| `AVOID_OBSTACLE` | `max(IR) > 80` | Rotate away from strongest reading | `max(IR) < 48` → `PATROL` |
| `EMERGENCY_STOP` | Front IR `> 350` AND a side sensor also high | Full stop, blinking LEDs | Front clears → `RECOVERY` |
| `RECOVERY` | Triggered by `EMERGENCY_STOP` or stall timer | Reverse 0.8 s, then rotate 1.2 s | Time elapsed → `PATROL` |

---

## Mapping to lecture material

The implementation tracks the lectures the course actually delivered:

- **Wk04 Embedded AI** — FSM (`class FSM`, switch-style state handler), Three-Zone priority logic, moving-average noise filter (`_ds_history`), sensor fusion (IR + camera in `classify_threat`), fail-safe default-to-safety (`EMERGENCY_STOP` / `RECOVERY`).
- **Wk05 Webots intro** — Standard Sense→Think→Act loop, `getDevice` / `enable(TIME_STEP)`, exact E-puck device names, `OBSTACLE_THRESHOLD = 80`, project file layout (`worlds/`, `controllers/<name>/`).
- **Wk06 Autonomous Navigation** — Reactive + planned navigation hybrid (the patrol cycle is the planned layer; obstacle avoidance is the reactive layer).
- **Wk07 Vision & Perception** — RGB threshold rule (`R>200 ∧ G<80 ∧ B<80`) used verbatim from the lecture slide; centroid-based object tracking (`x_accum / count`); colour pipeline `Camera image → threshold → binary mask → centroid → decision`.

---

## Code organisation (Wk04 / spec §6 "modular and structured")

`scope_controller.py` is split into six clearly-numbered sections:

1. Robot initialisation (devices)
2. Configuration constants (every threshold lives here, not buried in logic)
3. `# SENSE` — perception helpers (`read_distance_sensors`, `perceive_vision`, `proximity_summary`)
4. `# ACT` — actuation helpers (`drive`, `stop`, `set_alert_leds`)
5. `# THINK` — FSM + classifier (`class FSM`, `classify_threat`)
6. Main loop with explicit `# SENSE` / `# THINK` / `# ACT` markers

Every magic number is a named constant with a comment explaining its origin (lecture slide, sensor characteristic, etc.).

---

## Demo plan (matches video rubric, 5 min total)

- **0:00–1:00 — Introduction.** Each member introduces themselves and the project goal.
- **1:00–4:00 — Demonstration.**
  - Start sim → robot patrols, console logs `STATE -> PATROL`.
  - Robot enters RZ1, sees red intruder + zone floor → `INTRUDER_DETECTED (threat=HIGH)` → `NAVIGATE_TO_TARGET` (LEDs blink) → `INTERCEPT` → `ALERT`.
  - Drag a wall in front of the robot during patrol → `EMERGENCY_STOP` → `RECOVERY` → back to `PATROL`. Demonstrates safety override.
  - Move the intruder out of view → `SCAN` rotates the robot → resumes patrol when threat clears.
- **4:00–5:00 — Code & architecture.** Show the FSM class, the priority chain in the main loop, the `classify_threat` sensor-fusion function, and the architecture diagram in this README.

---

## Submission checklist

- [x] Source code (this directory) — push to private GitHub repo, invite the three teaching staff assessors as collaborators.
- [ ] Final 4-page report following the provided template, citing this repo URL.
- [ ] 5-minute video where every member speaks and the system is demonstrated.

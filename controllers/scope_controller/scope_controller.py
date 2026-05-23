"""
SCOPE(Smart Continuous Optical Patrol Engine)-X
Autonomous security patrol robot — Webots E-puck platform.
Architecture: Sense -> Think -> Act, prioritised FSM.

Group NA9: Vraj Patel, Vo Duc Anh Tran, Manveer Singh
Course: 3003ICT - Programming for Robotics
"""

import math
from controller import Robot



# SECTION 1 — ROBOT INITIALISATION  [VRAJ PATEL]


robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

distance_sensors = []
for i in range(8):
    ds = robot.getDevice(f"ps{i}")
    ds.enable(TIME_STEP)
    distance_sensors.append(ds)

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
CAM_W = camera.getWidth()
CAM_H = camera.getHeight()

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

iu = robot.getDevice("iu")
iu.enable(TIME_STEP)

speaker = robot.getDevice("speaker")
if speaker is not None:
    try:
        speaker.setEngine("native")
    except Exception:
        pass
    try:
        speaker.setLanguage("en-US")
    except Exception:
        pass

leds = []
for i in range(10):
    try:
        led = robot.getDevice(f"led{i}")
        if led is not None:
            leds.append(led)
    except Exception:
        pass



# SECTION 2 — CONFIGURATION CONSTANTS  [MANVEER SINGH]

MAX_SPEED        = 6.28
PATROL_SPEED     = 4.0
PATROL_SPEED_LOW = 2.5    # cautious speed when LOW threat detected
TURN_SPEED       = 3.0

# IR thresholds (E-puck range: 0 = clear, ~4000 = contact)
# 300 chosen because corridor walls read 80-150 at normal clearance

OBSTACLE_THRESHOLD  = 300.0
EMERGENCY_THRESHOLD = 1800.0
INTERCEPT_DISTANCE  = 150.0

# Wk07 RGB rule: R > 200 AND G < 80 AND B < 80
RED_R_MIN, RED_G_MAX, RED_B_MAX = 200, 80, 80
RED_PIXEL_RATIO_DETECT = 0.008
RED_PIXEL_RATIO_CLOSE  = 0.07

WAYPOINT_TOLERANCE = 0.15   # metres
HEADING_TOLERANCE  = 0.15   # radians
HEADING_KP         = 3.5

# Lawnmower patrol — 3 N-S strips per room
WAYPOINTS = [
    # TOP ROOMS
    ( 0.00,  0.42),
    ( 0.00,  0.58),
    (-1.20,  0.58),
    (-1.20,  1.00),
    (-0.85,  1.00),
    (-0.85,  0.58),
    (-0.55,  0.58),
    (-0.55,  1.00),
    (-0.55,  0.58),
    ( 0.55,  0.58),
    ( 0.55,  1.00),
    ( 0.85,  1.00),
    ( 0.85,  0.58),
    ( 1.20,  0.58),
    ( 1.20,  1.00),
    ( 1.20,  0.58),
    # CORRIDOR
    ( 0.00,  0.42),
    (-1.20,  0.00),
    ( 1.20,  0.00),
    ( 0.00,  0.00),
    # BOTTOM ROOMS
    ( 0.00, -0.42),
    ( 0.00, -0.58),
    ( 1.20, -0.58),
    ( 1.20, -1.00),
    ( 0.85, -1.00),
    ( 0.85, -0.58),
    ( 0.55, -0.58),
    ( 0.55, -1.00),
    ( 0.55, -0.58),
    (-0.50, -0.58),  # BL threshold — triggers NO_ENTRY_ZONE
    # LOOP BACK
    ( 0.00, -0.42),
    ( 0.00,  0.00),
]

STALL_DETECT_MS     = 1500
RECOVERY_REVERSE_MS = 800
RECOVERY_TURN_MS    = 1200
DS_FILTER_WINDOW    = 5
RZ_BUFFER           = 0.40
THREAT_LOG_DEBOUNCE_MS = 1000

# SECTION 3a — SENSE: IR Sensor Perception  [VO DUC ANH TRAN]

_ds_history = [[] for _ in range(8)]


def read_distance_sensors():
    smoothed = []
    for i, s in enumerate(distance_sensors):
        raw = s.getValue()
        buf = _ds_history[i]
        buf.append(raw)
        if len(buf) > DS_FILTER_WINDOW:
            buf.pop(0)
        smoothed.append(sum(buf) / len(buf))
    return smoothed


def proximity_summary(values):
    # Only front-arc sensors used — avoids false triggers from corridor walls
    front_max = max(values[0], values[1], values[6], values[7])
    return {
        "front":       max(values[0], values[7]),
        "front_left":  values[6],
        "front_right": values[1],
        "side_left":   values[5],
        "side_right":  values[2],
        "max":         max(values),
        "front_max":   front_max,
    }


def in_restricted_zone(gps_values):
    # RZ1 (TR server room): x in [0.38, 1.45], y in [0.38, 1.45]
    x, y = gps_values[0], gps_values[1]
    return 0.38 <= x <= 1.45 and 0.38 <= y <= 1.45


def near_restricted_zone(gps_values):
    # Euclidean distance to nearest point on RZ1 rectangle
    if in_restricted_zone(gps_values):
        return False
    x, y = gps_values[0], gps_values[1]
    cx = max(0.38, min(1.45, x))
    cy = max(0.38, min(1.45, y))
    return math.hypot(x - cx, y - cy) < RZ_BUFFER



# SECTION 3b — SENSE: Camera Vision  [VO DUC ANH TRAN]

def perceive_vision():
    """Detect red intruder using Wk07 RGB threshold + centroid tracking."""
    image = camera.getImage()
    if image is None:
        return {"red_ratio": 0.0, "intruder_x": None}

    red_count   = 0
    red_x_accum = 0

    for py in range(0, CAM_H, 2):
        for px in range(0, CAM_W, 2):
            r = camera.imageGetRed(image,   CAM_W, px, py)
            g = camera.imageGetGreen(image, CAM_W, px, py)
            b = camera.imageGetBlue(image,  CAM_W, px, py)
            if r > RED_R_MIN and g < RED_G_MAX and b < RED_B_MAX:
                red_count   += 1
                red_x_accum += px

    sampled    = (CAM_W // 2) * (CAM_H // 2)
    red_ratio  = red_count / sampled if sampled else 0.0
    intruder_x = (red_x_accum / red_count) if red_count > 0 else None

    return {"red_ratio": red_ratio, "intruder_x": intruder_x}



# SECTION 3c — SENSE: Navigation Geometry  [MANVEER SINGH]

def get_heading():
    return iu.getRollPitchYaw()[2]


def angle_diff(a, b):
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def target_bearing(rx, ry, tx, ty):
    return math.atan2(ty - ry, tx - rx)



# SECTION 4 — ACT: Actuation Helpers  [MANVEER SINGH]

def drive(left, right):
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))


def stop():
    drive(0.0, 0.0)


def set_alert_leds(on, pattern="solid"):
    for i, led in enumerate(leds):
        if not on:
            led.set(0)
        elif pattern == "solid":
            led.set(1)
        elif pattern == "blink":
            t = int(robot.getTime() * 4)
            led.set(1 if (i + t) % 2 == 0 else 0)


def speak(msg):
    print(f"[SCOPE-X] SPEAKING: \"{msg}\"", flush=True)
    if speaker is not None:
        try:
            speaker.speak(msg, 1.0)
        except Exception as e:
            print(f"[SCOPE-X] Speaker error: {e}", flush=True)



# SECTION 5a — THINK: FSM Architecture  [VRAJ PATEL]

class FSM:
    PATROL            = "PATROL"
    SCAN              = "SCAN"
    INTRUDER_DETECTED = "INTRUDER_DETECTED"
    NAVIGATE_TO_TARGET= "NAVIGATE_TO_TARGET"
    INTERCEPT         = "INTERCEPT"
    ALERT             = "ALERT"
    AVOID_OBSTACLE    = "AVOID_OBSTACLE"
    EMERGENCY_STOP    = "EMERGENCY_STOP"
    RECOVERY          = "RECOVERY"
    NO_ENTRY_ZONE     = "NO_ENTRY_ZONE"


# =====================================================================
# SECTION 5b — THINK: Context-Aware Threat Classifier  [VRAJ PATEL]
#              ** ADVANCED COMPONENT **
# =====================================================================

def classify_threat(vision, in_rz, near_rz):
    """
    Fuses camera (object presence) and GPS (spatial context) into a
    graded threat level — same object gets different severity depending
    on where the robot is in the arena.

      NONE   — no red pixels
      LOW    — red visible, far from RZ (monitor only)
      MEDIUM — red visible, near RZ boundary, watch closely and follow
      HIGH   — red visible, inside restricted zone, ALERT
    """
    red = vision["red_ratio"]
    if red < RED_PIXEL_RATIO_DETECT:
        return "NONE"
    if in_rz:
        return "HIGH"
    if near_rz:
        return "MEDIUM"
    return "LOW"


# ---- state variables ----
state                = FSM.PATROL
last_state           = None
last_threat          = None
last_threat_log_time = -THREAT_LOG_DEBOUNCE_MS
scan_timer           = 0
alert_timer          = 0
stall_timer          = 0
recovery_phase       = "reverse"
recovery_timer       = 0
waypoint_idx         = 0
alert_announced      = False
avoid_timer          = 0
AVOID_TIMEOUT_MS     = 2500
emergency_timer      = 0

# Obstacle avoidance phases: turn 90° -> slide forward -> realign to original heading
AVOID_TURN_MS       = 600
AVOID_SLIDE_MS      = 1000
avoid_phase         = "turn"
avoid_saved_heading = 0.0
avoid_turn_dir      = 1
avoid_slide_timer   = 0

prev_in_rz          = False
rz_entry_scanned    = False

BL_NOGO_X           = -0.42
BL_NOGO_Y           = -0.42
no_entry_timer      = 0
no_entry_announced  = False
no_entry_triggered  = False



# SECTION 6 — MAIN LOOP  (Sense -> Think -> Act)

print("[SCOPE-X] Boot complete. Beginning autonomous patrol.", flush=True)

while robot.step(TIME_STEP) != -1:

   
    # SENSE  [Vo Duc Anh Tran]
   
    ds_values  = read_distance_sensors()
    prox       = proximity_summary(ds_values)
    vision     = perceive_vision()
    gps_values = gps.getValues()
    in_rz      = in_restricted_zone(gps_values)
    near_rz    = near_restricted_zone(gps_values)
    threat     = classify_threat(vision, in_rz, near_rz)
    now_ms     = robot.getTime() * 1000

  
    # THINK — Priority Chain  [Vraj Patel]
    # Safety > Mission > Reflex > Default
   
    front_blocked_hard = prox["front"] > EMERGENCY_THRESHOLD and (
        prox["front_left"] > OBSTACLE_THRESHOLD or prox["front_right"] > OBSTACLE_THRESHOLD
    )

    if front_blocked_hard and state != FSM.RECOVERY:
        if state != FSM.EMERGENCY_STOP:
            emergency_timer = 0
        state = FSM.EMERGENCY_STOP

    elif threat in ("MEDIUM", "HIGH") and state in (FSM.PATROL, FSM.SCAN):
        # Intruder detection takes priority over obstacle avoidance
        state = FSM.INTRUDER_DETECTED
        scan_timer = 0

    elif prox["front_max"] > OBSTACLE_THRESHOLD and state not in (
        FSM.EMERGENCY_STOP, FSM.RECOVERY, FSM.AVOID_OBSTACLE,
        FSM.NAVIGATE_TO_TARGET, FSM.INTERCEPT, FSM.INTRUDER_DETECTED,
        FSM.NO_ENTRY_ZONE,
    ):
        avoid_saved_heading = get_heading()
        avoid_turn_dir      = 1 if prox["front_left"] <= prox["front_right"] else -1
        avoid_phase         = "turn"
        avoid_slide_timer   = 0
        state = FSM.AVOID_OBSTACLE
        avoid_timer = 0

    if state != last_state:
        print(f"[SCOPE-X] State -> {state}  threat={threat} "
              f"red={vision['red_ratio']:.3f} in_rz={in_rz} near_rz={near_rz} "
              f"pos=({gps_values[0]:.2f},{gps_values[1]:.2f})",
              flush=True)
        last_state = state

    if threat != last_threat and (now_ms - last_threat_log_time) >= THREAT_LOG_DEBOUNCE_MS:
        print(f"[SCOPE-X] Threat -> {threat}  "
              f"red={vision['red_ratio']:.3f} in_rz={in_rz} near_rz={near_rz}",
              flush=True)
        last_threat = threat
        last_threat_log_time = now_ms


    # ACT — Safety States  [Manveer Singh]
    
    if state == FSM.EMERGENCY_STOP:
        stop()
        set_alert_leds(True, "blink")
        emergency_timer += TIME_STEP
        if prox["front"] < OBSTACLE_THRESHOLD or emergency_timer > 800:
            state = FSM.RECOVERY
            recovery_phase = "reverse"
            recovery_timer = 0
            emergency_timer = 0

    elif state == FSM.RECOVERY:
        recovery_timer += TIME_STEP
        if recovery_phase == "reverse":
            drive(-TURN_SPEED, -TURN_SPEED)
            if recovery_timer >= RECOVERY_REVERSE_MS:
                recovery_phase = "rotate"
                recovery_timer = 0
        else:
            drive(-TURN_SPEED, TURN_SPEED)
            if recovery_timer >= RECOVERY_TURN_MS:
                set_alert_leds(False)
                state = FSM.PATROL

    elif state == FSM.AVOID_OBSTACLE:
        # turn -> slide -> align back to saved heading
        avoid_timer += TIME_STEP

        if avoid_phase == "turn":
            target_h = avoid_saved_heading + avoid_turn_dir * math.pi / 2
            herr = angle_diff(target_h, get_heading())
            if abs(herr) < HEADING_TOLERANCE or avoid_timer > AVOID_TURN_MS:
                avoid_phase = "slide"
                avoid_slide_timer = 0
            else:
                sign = 1.0 if herr > 0 else -1.0
                drive(-sign * TURN_SPEED, sign * TURN_SPEED)

        elif avoid_phase == "slide":
            drive(PATROL_SPEED, PATROL_SPEED)
            avoid_slide_timer += TIME_STEP
            if (prox["front_max"] < OBSTACLE_THRESHOLD * 0.5
                    and avoid_slide_timer > 300) or avoid_slide_timer > AVOID_SLIDE_MS:
                avoid_phase = "align"

        elif avoid_phase == "align":
            herr = angle_diff(avoid_saved_heading, get_heading())
            if abs(herr) < HEADING_TOLERANCE:
                avoid_timer = 0
                state = FSM.PATROL
            else:
                sign = 1.0 if herr > 0 else -1.0
                drive(-sign * TURN_SPEED, sign * TURN_SPEED)

        if state == FSM.AVOID_OBSTACLE and avoid_timer > AVOID_TIMEOUT_MS * 2:
            waypoint_idx = (waypoint_idx + 1) % len(WAYPOINTS)
            avoid_timer  = 0
            state = FSM.PATROL

    # ACT — Mission States  [Vraj Patel]
   
    elif state == FSM.INTRUDER_DETECTED:
        stop()
        set_alert_leds(True, "solid")
        scan_timer += TIME_STEP
        if scan_timer > 600:
            scan_timer = 0
            if threat in ("HIGH", "MEDIUM"):
                state = FSM.NAVIGATE_TO_TARGET
            else:
                state = FSM.PATROL
                set_alert_leds(False)

    # ACT — Navigation States  [Vo Duc Anh Tran]
   
    elif state == FSM.NAVIGATE_TO_TARGET:
        # Outside RZ1: GPS navigation to doorway, then switch to camera servo
        # Inside  RZ1: camera centroid servoing to track intruder
        set_alert_leds(True, "blink")
        if vision["intruder_x"] is None:
            state = FSM.SCAN
            scan_timer = 0
        elif threat == "NONE":
            state = FSM.PATROL
            set_alert_leds(False)
        elif not in_rz:
            rx, ry = gps_values[0], gps_values[1]
            entry  = (0.55, 0.58)
            dist   = math.hypot(entry[0] - rx, entry[1] - ry)
            if dist < WAYPOINT_TOLERANCE:
                err = (vision["intruder_x"] - CAM_W / 2) / (CAM_W / 2)
                drive(PATROL_SPEED - err * TURN_SPEED, PATROL_SPEED + err * TURN_SPEED)
            else:
                cur_h = get_heading()
                tgt_h = target_bearing(rx, ry, entry[0], entry[1])
                herr  = angle_diff(tgt_h, cur_h)
                if abs(herr) > HEADING_TOLERANCE:
                    sign = 1.0 if herr > 0 else -1.0
                    drive(-sign * TURN_SPEED, sign * TURN_SPEED)
                else:
                    steer = max(-1.0, min(1.0, herr * HEADING_KP))
                    drive(PATROL_SPEED * (1 - steer), PATROL_SPEED * (1 + steer))
            if vision["red_ratio"] >= RED_PIXEL_RATIO_CLOSE or prox["front"] > INTERCEPT_DISTANCE:
                state = FSM.INTERCEPT
                alert_timer = 0
        else:
            err = (vision["intruder_x"] - CAM_W / 2) / (CAM_W / 2)
            drive(PATROL_SPEED - err * TURN_SPEED, PATROL_SPEED + err * TURN_SPEED)
            if vision["red_ratio"] >= RED_PIXEL_RATIO_CLOSE or prox["front"] > INTERCEPT_DISTANCE:
                state = FSM.INTERCEPT
                alert_timer = 0

    elif state == FSM.INTERCEPT:
        set_alert_leds(True, "blink")
        if vision["red_ratio"] < RED_PIXEL_RATIO_DETECT:
            state = FSM.SCAN
            scan_timer = 0
        elif vision["red_ratio"] < RED_PIXEL_RATIO_CLOSE * 0.5:
            state = FSM.NAVIGATE_TO_TARGET
        else:
            if prox["front"] > INTERCEPT_DISTANCE:
                drive(-TURN_SPEED, -TURN_SPEED)
            elif prox["front"] < INTERCEPT_DISTANCE * 0.5:
                drive(1.0, 1.0)
            else:
                stop()
            alert_timer += TIME_STEP
            if alert_timer > 1500:
                alert_timer = 0
                state = FSM.ALERT

    elif state == FSM.SCAN:
        drive(-TURN_SPEED * 0.7, TURN_SPEED * 0.7)
        scan_timer += TIME_STEP
        if vision["red_ratio"] >= RED_PIXEL_RATIO_DETECT:
            state = FSM.INTRUDER_DETECTED
            scan_timer = 0
        elif scan_timer > 3000:
            scan_timer = 0
            state = FSM.PATROL
 
    # ACT — Alert State  [Vraj Patel]
  
    elif state == FSM.ALERT:
        set_alert_leds(True, "blink")
        stop()
        alert_timer += TIME_STEP

        if not alert_announced:
            if in_rz:
                msg = "Alert. Intruder confirmed inside the server room."
                print("[SCOPE-X] >>> ALERT HIGH <<<  Intruder INSIDE server room (RZ1)", flush=True)
            else:
                msg = "Warning. Intruder detected approaching the server room."
                print("[SCOPE-X] >>> ALERT MEDIUM <<<  Intruder approaching server room (RZ1)", flush=True)
            speak(msg)
            alert_announced = True

        if threat in ("MEDIUM", "HIGH") and vision["intruder_x"] is not None and alert_timer > 2000:
            alert_timer = 0
            alert_announced = False
            state = FSM.NAVIGATE_TO_TARGET
        elif threat in ("NONE", "LOW") or alert_timer > 6000:
            set_alert_leds(False)
            state = FSM.PATROL
            alert_announced = False
            alert_timer = 0
            print("[SCOPE-X] Alert cleared -> resuming patrol", flush=True)

    # ACT — Patrol & Access Control  [Manveer Singh]
   
    elif state == FSM.NO_ENTRY_ZONE:
        stop()
        set_alert_leds(True, "blink")
        no_entry_timer += TIME_STEP
        if not no_entry_announced:
            speak("Warning. No entry zone. Turning back.")
            print("[SCOPE-X] >>> NO-ENTRY ZONE <<<  BL room is off-limits — turning back", flush=True)
            no_entry_announced = True
        if no_entry_timer > 2500:
            while (WAYPOINTS[waypoint_idx][0] < BL_NOGO_X and
                   WAYPOINTS[waypoint_idx][1] < BL_NOGO_Y):
                waypoint_idx = (waypoint_idx + 1) % len(WAYPOINTS)
            no_entry_timer = 0
            no_entry_announced = False
            set_alert_leds(False)
            state = FSM.PATROL

    else:  # FSM.PATROL
        set_alert_leds(False)

        if (gps_values[0] < BL_NOGO_X and gps_values[1] < BL_NOGO_Y
                and not no_entry_triggered):
            no_entry_triggered = True
            no_entry_timer = 0
            no_entry_announced = False
            state = FSM.NO_ENTRY_ZONE

        elif in_rz and not prev_in_rz and not rz_entry_scanned:
            print(f"[SCOPE-X] Entering RZ1 — rotating scan  "
                  f"pos=({gps_values[0]:.2f},{gps_values[1]:.2f})", flush=True)
            rz_entry_scanned = True
            state = FSM.SCAN
            scan_timer = 0

        else:
            speed = PATROL_SPEED_LOW if threat == "LOW" else PATROL_SPEED
            target_x, target_y = WAYPOINTS[waypoint_idx]
            rx, ry = gps_values[0], gps_values[1]
            dist   = math.hypot(target_x - rx, target_y - ry)

            if dist < WAYPOINT_TOLERANCE:
                waypoint_idx = (waypoint_idx + 1) % len(WAYPOINTS)
            else:
                cur_h = get_heading()
                tgt_h = target_bearing(rx, ry, target_x, target_y)
                herr  = angle_diff(tgt_h, cur_h)
                if abs(herr) > HEADING_TOLERANCE:
                    sign = 1.0 if herr > 0 else -1.0
                    drive(-sign * TURN_SPEED, sign * TURN_SPEED)
                else:
                    steer = max(-1.0, min(1.0, herr * HEADING_KP))
                    drive(speed * (1 - steer), speed * (1 + steer))

    # Stall detection watchdog  [Manveer Singh]
    
    moving_cmd = abs(left_motor.getVelocity()) + abs(right_motor.getVelocity()) > 0.5
    if moving_cmd and prox["front"] > EMERGENCY_THRESHOLD:
        stall_timer += TIME_STEP
        if stall_timer > STALL_DETECT_MS and state not in (FSM.RECOVERY, FSM.EMERGENCY_STOP):
            print("[SCOPE-X] Stall detected -> RECOVERY", flush=True)
            state = FSM.RECOVERY
            recovery_phase = "reverse"
            recovery_timer = 0
            stall_timer = 0
    else:
        stall_timer = max(0, stall_timer - TIME_STEP)

    # bookkeeping
    if not in_rz:
        rz_entry_scanned = False
    prev_in_rz = in_rz

    if gps_values[0] >= BL_NOGO_X or gps_values[1] >= BL_NOGO_Y:
        no_entry_triggered = False

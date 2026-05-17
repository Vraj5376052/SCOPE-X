"""
SCOPE(Smart Continuous Optical Patrol Engine)-X
===============================================
Autonomous security patrol robot built on the Webots E-puck platform.

Architecture: Sense -> Think -> Act, driven by a prioritised Finite State
Machine (FSM). The robot patrols an environment, classifies intrusions
using context-aware perception (object presence + spatial zone), and
escalates behaviour from passive monitoring to active interception.

Group: NA9 - Vraj Patel, Vo Duc Anh Tran, Manveer Singh
Course: 3003ICT - Programming for Robotics
"""

import math
from controller import Robot

# =====================================================================
# 1. ROBOT INITIALISATION
# =====================================================================
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# ---------- Motors ----------
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ---------- Distance / proximity sensors (ps0..ps7) ----------
distance_sensors = []
for i in range(8):
    ds = robot.getDevice(f"ps{i}")
    ds.enable(TIME_STEP)
    distance_sensors.append(ds)

# ---------- Camera (vision-based perception) ----------
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
CAM_W = camera.getWidth()
CAM_H = camera.getHeight()

# ---------- GPS (zone-aware context) ----------
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

# ---------- InertialUnit (heading for waypoint navigation) ----------
iu = robot.getDevice("iu")
iu.enable(TIME_STEP)

# ---------- Speaker (audible intruder alert) ----------
speaker = robot.getDevice("speaker")
if speaker is not None:
    try:
        speaker.setLanguage("en-US")
    except Exception:
        pass

# ---------- LEDs (alert / status indication) ----------
leds = []
for i in range(10):  # E-puck has up to 10 LEDs (led0..led9)
    try:
        led = robot.getDevice(f"led{i}")
        if led is not None:
            leds.append(led)
    except Exception:
        pass

# =====================================================================
# 2. CONFIGURATION CONSTANTS
# =====================================================================
MAX_SPEED = 6.28           # E-puck max wheel speed (rad/s)
PATROL_SPEED = 4.0
TURN_SPEED = 3.0
INTERCEPT_SPEED = 5.0

# Proximity thresholds (E-puck IR returns ~0 far, ~4000 very close)
# Wk05 lecture: ps0/ps7 > 80 -> obstacle (the standard E-puck threshold)
OBSTACLE_THRESHOLD = 300.0    # triggers avoidance ~0.20 m from obstacle
EMERGENCY_THRESHOLD = 1800.0  # triggers emergency stop ~0.07 m (truly imminent)
INTERCEPT_DISTANCE = 200.0

# Vision colour rules (Wk07 lecture, slide "RGB Color Channels"):
#   if R > 200 and G < 80 and B < 80: red_detected = True
RED_R_MIN, RED_G_MAX, RED_B_MAX = 200, 80, 80
ZONE_R_MIN, ZONE_R_MAX = 60, 180   # dark-red restricted-zone floor
ZONE_GB_MAX = 60

# Pixel-ratio thresholds for "is the most common colour Red?" (Wk05 slide 16)
RED_PIXEL_RATIO_DETECT = 0.015
RED_PIXEL_RATIO_CLOSE = 0.10
ZONE_PIXEL_RATIO = 0.08

# Waypoint-based patrol (Wk06 "Autonomous Navigation - Path Planning"):
#
# 3x3 arena layout:
#   Horizontal dividers at y=+/-0.42, doorway gap x in (-0.40, 0.40).
#   Vertical dividers at x=+/-0.45, spanning y in [0.68,1.47] / [-1.47,-0.68].
#   Junction strip y in [0.48, 0.68]: robot transits E-W here freely.
#
# Each room is swept with 3 N-S parallel strips 0.35 m apart (lawnmower).
# Strip spacing < camera FOV width at 0.4 m (~0.43 m) -> full coverage.
# Strip geometry for 3x3 arena (walls at x,y = ±1.50):
#   N/S strip endpoints at y = ±1.20  (0.30 m clearance from wall)
#   E/W strip endpoints at x = ±1.20  (0.30 m clearance from wall)
#   Middle column at ±0.85
#   Inner edge at ±0.55  (just inside divider)
#   Junction transit height: y = 0.58 (above horiz. divider, below vert. divider)
WAYPOINTS = [
    # ===== TOP ROOMS =====
    ( 0.00,  0.42),  # approach top doorway
    ( 0.00,  0.58),  # through doorway -> junction strip

    # -- TL room: 3 N-S strips at x = -1.20, -0.85, -0.55 --
    (-1.20,  0.58),  # junction: step west to strip-1
    (-1.20,  1.00),  # strip 1 -> north end  (0.50 m from north wall)
    (-0.85,  1.00),  # step east at top
    (-0.85,  0.58),  # strip 2 -> south end
    (-0.55,  0.58),  # step east (below vert. divider start y=0.68)
    (-0.55,  1.00),  # strip 3 -> north end  (inner edge of TL)
    (-0.55,  0.58),  # strip 3 -> south, return to junction

    # transit east through junction to TR/RZ1
    ( 0.55,  0.58),  # step east (below vert. divider, free transit)

    # -- TR/RZ1: 3 N-S strips at x = 0.55, 0.85, 1.20 --
    ( 0.55,  1.00),  # strip 1 -> north (inner edge of RZ1)
    ( 0.85,  1.00),  # step east at top
    ( 0.85,  0.58),  # strip 2 -> south (RZ1 centre column)
    ( 1.20,  0.58),  # step east (0.50 m from east wall)
    ( 1.20,  1.00),  # strip 3 -> north (east wall side)
    ( 1.20,  0.58),  # strip 3 -> south, return to junction

    # ===== CORRIDOR SWEEP =====
    ( 0.00,  0.42),  # exit top doorway
    (-1.20,  0.00),  # corridor west boundary
    ( 1.20,  0.00),  # corridor east boundary
    ( 0.00,  0.00),  # corridor centre

    # ===== BOTTOM ROOMS =====
    ( 0.00, -0.42),  # approach bottom doorway
    ( 0.00, -0.58),  # through doorway -> bottom junction

    # -- BR room: 3 N-S strips at x = 1.20, 0.85, 0.55 --
    ( 1.20, -0.58),  # step east to strip-1
    ( 1.20, -1.00),  # strip 1 -> south (0.50 m from south wall)
    ( 0.85, -1.00),  # step west at bottom
    ( 0.85, -0.58),  # strip 2 -> north
    ( 0.55, -0.58),  # step west
    ( 0.55, -1.00),  # strip 3 -> south (inner edge of BR)
    ( 0.55, -0.58),  # strip 3 -> north, return to junction

    # transit west through junction to BL/RZ2
    (-0.55, -0.58),  # step west (below vert. divider)

    # -- BL/RZ2: 3 N-S strips at x = -0.55, -0.85, -1.20 --
    (-0.55, -1.00),  # strip 1 -> south (inner edge of RZ2)
    (-0.85, -1.00),  # step west at bottom
    (-0.85, -0.58),  # strip 2 -> north (RZ2 centre column)
    (-1.20, -0.58),  # step west (0.50 m from west wall)
    (-1.20, -1.00),  # strip 3 -> south (west wall side)
    (-1.20, -0.58),  # strip 3 -> north, return to junction

    # ===== BACK TO START -> LOOP =====
    ( 0.00, -0.42),  # exit bottom doorway
    ( 0.00,  0.00),  # corridor centre -> loop
]
WAYPOINT_TOLERANCE = 0.15   # metres - waypoint considered reached
HEADING_TOLERANCE  = 0.15   # rad (~8.6 deg) - aligned enough to drive
HEADING_KP         = 3.5    # proportional steering gain when driving

# Stall / recovery (Wk05 reliability slide: "Recovers from Unexpected Input")
STALL_DETECT_MS = 1500
RECOVERY_REVERSE_MS = 800
RECOVERY_TURN_MS = 1200

# Moving-average window for distance sensors
# (Wk04 "Example AI Enhancement #2 - Sensor Averaging / Noise Reduction")
DS_FILTER_WINDOW = 5

# =====================================================================
# 3. SENSE - perception helpers
# =====================================================================
# Rolling buffer of recent IR readings, one ring buffer per sensor.
# Implements Wk04 "Moving Average Filter" for noise reduction.
_ds_history = [[] for _ in range(8)]


def read_distance_sensors():
    """Read 8 IR sensors, apply moving-average noise filter, return smoothed values."""
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
    front = max(values[0], values[7])
    # front_max: only front-arc sensors (ps0, ps1, ps6, ps7) used for obstacle
    # triggering — prevents side wall readings from causing false AVOID_OBSTACLE
    front_max = max(values[0], values[1], values[6], values[7])
    return {
        "front": front,
        "front_left": values[6],
        "front_right": values[1],
        "side_left": values[5],
        "side_right": values[2],
        "max": max(values),
        "front_max": front_max,
    }


def in_restricted_zone(gps_values):
    """Return True if robot is INSIDE a restricted zone.
    Bounds expanded 0.10 m beyond physical floor marker so detection
    fires as the robot reaches the zone edge, not after it has entered.
      RZ1 (TR room): x ∈ [0.38, 1.45], y ∈ [0.38, 1.45]
      RZ2 (BL room): x ∈ [-1.45,-0.38], y ∈ [-1.45,-0.38]
    """
    x, y = gps_values[0], gps_values[1]
    rz1 = (0.38 <= x <= 1.45 and 0.38 <= y <= 1.45)
    rz2 = (-1.45 <= x <= -0.38 and -1.45 <= y <= -0.38)
    return rz1 or rz2


RZ_BUFFER = 0.35   # metres outside the RZ boundary that counts as "near"

def near_restricted_zone(gps_values):
    """Return True if robot is within RZ_BUFFER metres of a restricted zone
    but NOT yet inside it (pitch: MEDIUM threat zone).
    """
    if in_restricted_zone(gps_values):
        return False
    x, y = gps_values[0], gps_values[1]
    near_rz1 = ((0.48 - RZ_BUFFER) <= x <= 1.45 and
                (0.48 - RZ_BUFFER) <= y <= 1.45)
    near_rz2 = (-1.45 <= x <= (-0.48 + RZ_BUFFER) and
                -1.45 <= y <= (-0.48 + RZ_BUFFER))
    return near_rz1 or near_rz2


def get_heading():
    """Return robot yaw in radians (rotation around world +Z, ENU)."""
    rpy = iu.getRollPitchYaw()
    return rpy[2]


def angle_diff(a, b):
    """Shortest signed difference (a - b) wrapped to [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def target_bearing(rx, ry, tx, ty):
    """World bearing from (rx,ry) to (tx,ty) in ENU yaw convention.
    yaw=0 -> facing +X (east); yaw=+pi/2 -> facing +Y (north).
    """
    return math.atan2(ty - ry, tx - rx)


def perceive_vision():
    """Sample the camera image for the red intruder and dark-red zone floor."""
    image = camera.getImage()
    if image is None:
        return {"red_ratio": 0.0, "zone_ratio": 0.0, "intruder_x": None}

    red_count = 0
    zone_count = 0
    red_x_accum = 0

    for y in range(0, CAM_H, 2):
        for x in range(0, CAM_W, 2):
            r = camera.imageGetRed(image, CAM_W, x, y)
            g = camera.imageGetGreen(image, CAM_W, x, y)
            b = camera.imageGetBlue(image, CAM_W, x, y)

            # Wk07 colour rule: bright red intruder
            if r > RED_R_MIN and g < RED_G_MAX and b < RED_B_MAX:
                red_count += 1
                red_x_accum += x
            # Dark-red restricted zone floor (lower intensity, dominant red)
            elif ZONE_R_MIN < r < ZONE_R_MAX and g < ZONE_GB_MAX and b < ZONE_GB_MAX:
                zone_count += 1

    sampled = (CAM_W // 2) * (CAM_H // 2)
    red_ratio = red_count / sampled if sampled else 0
    zone_ratio = zone_count / sampled if sampled else 0
    intruder_x = (red_x_accum / red_count) if red_count > 0 else None

    return {"red_ratio": red_ratio, "zone_ratio": zone_ratio, "intruder_x": intruder_x}


# =====================================================================
# 4. ACT - actuation helpers
# =====================================================================
def drive(left, right):
    left = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)


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


# =====================================================================
# 5. THINK - FSM
# =====================================================================
class FSM:
    PATROL = "PATROL"
    SCAN = "SCAN"
    INTRUDER_DETECTED = "INTRUDER_DETECTED"
    NAVIGATE_TO_TARGET = "NAVIGATE_TO_TARGET"
    INTERCEPT = "INTERCEPT"
    ALERT = "ALERT"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RECOVERY = "RECOVERY"


state = FSM.PATROL
last_state = None
patrol_timer = 0
scan_timer = 0
alert_timer = 0
stall_timer = 0
recovery_phase = "reverse"
recovery_timer = 0
waypoint_idx = 0          # current target in WAYPOINTS
alert_announced = False   # speaker.speak() called once per intruder lock-on
avoid_timer = 0           # ms spent in current AVOID_OBSTACLE entry
AVOID_TIMEOUT_MS = 2500   # if avoiding for this long, skip current waypoint
emergency_timer = 0       # ms spent in EMERGENCY_STOP before forcing RECOVERY


def classify_threat(vision, in_rz, near_rz):
    """
    Context-aware threat classification matching the pitch document:
      NONE   - no red object detected by camera
      LOW    - red object detected, robot is outside and not near any RZ
      MEDIUM - red object detected AND robot is approaching / near an RZ boundary
      HIGH   - red object detected AND robot is inside a restricted zone

    Sensor fusion: camera red_ratio (object presence) + GPS zone context.
    The threat level rises as the robot moves closer to the restricted zone,
    giving the assessor a clear THREE-LEVEL classification demo.
    """
    red = vision["red_ratio"]
    if red < RED_PIXEL_RATIO_DETECT:
        return "NONE"
    if in_rz:
        return "HIGH"
    if near_rz:
        return "MEDIUM"
    return "LOW"


# =====================================================================
# 6. MAIN LOOP
# =====================================================================
print("[SCOPE-X] Boot complete. Beginning autonomous patrol.", flush=True)

while robot.step(TIME_STEP) != -1:
    # ----- SENSE -----
    ds_values = read_distance_sensors()
    prox = proximity_summary(ds_values)
    vision = perceive_vision()
    gps_values = gps.getValues()          # [x, y, z] world-frame position
    in_rz   = in_restricted_zone(gps_values)
    near_rz = near_restricted_zone(gps_values)
    threat  = classify_threat(vision, in_rz, near_rz)

    # ----- THINK: prioritised transitions -----
    # Wk04 "Three-Zone Logic" applied as a strict priority chain:
    #   Priority 1 (CRITICAL safety) -> Emergency stop
    #   Priority 2 (Reflex)          -> Obstacle avoidance
    #   Priority 3 (Mission)         -> Intruder response
    #   Priority 4 (Default)         -> Patrol
    # Sensor fusion: classify_threat combines vision (camera) + spatial context
    # (zone-floor pixels), and obstacle priority comes from IR proximity. The
    # safety/reflex layer cannot be overridden by mission states.
    front_blocked_hard = prox["front"] > EMERGENCY_THRESHOLD and (
        prox["front_left"] > OBSTACLE_THRESHOLD or prox["front_right"] > OBSTACLE_THRESHOLD
    )
    if front_blocked_hard and state != FSM.RECOVERY:
        if state != FSM.EMERGENCY_STOP:
            emergency_timer = 0   # reset on fresh entry
        state = FSM.EMERGENCY_STOP
    elif prox["front_max"] > OBSTACLE_THRESHOLD and state not in (
        FSM.EMERGENCY_STOP, FSM.RECOVERY, FSM.AVOID_OBSTACLE,
        FSM.NAVIGATE_TO_TARGET, FSM.INTERCEPT,
    ):
        state = FSM.AVOID_OBSTACLE
        avoid_timer = 0
    elif threat in ("MEDIUM", "HIGH") and state in (FSM.PATROL, FSM.SCAN):
        state = FSM.INTRUDER_DETECTED

    if state != last_state:
        print(f"[SCOPE-X] State -> {state}  threat={threat} "
              f"red={vision['red_ratio']:.3f} in_rz={in_rz} near_rz={near_rz} "
              f"pos=({gps_values[0]:.2f},{gps_values[1]:.2f})",
              flush=True)
        last_state = state

    # ----- ACT -----
    if state == FSM.EMERGENCY_STOP:
        stop()
        set_alert_leds(True, "blink")
        emergency_timer += TIME_STEP
        # Exit to RECOVERY when front clears OR after 800 ms timeout.
        # The timeout is essential: if the robot physically touched the obstacle
        # the sensor never self-clears until the robot reverses away.
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
                state = FSM.PATROL
                patrol_timer = 0
                set_alert_leds(False)

    elif state == FSM.AVOID_OBSTACLE:
        avoid_timer += TIME_STEP
        if prox["front_left"] > prox["front_right"]:
            drive(TURN_SPEED, -TURN_SPEED * 0.5)
        else:
            drive(-TURN_SPEED * 0.5, TURN_SPEED)
        if prox["front_max"] < OBSTACLE_THRESHOLD * 0.6:
            state = FSM.PATROL
            avoid_timer = 0
        elif avoid_timer > AVOID_TIMEOUT_MS:
            print(f"[SCOPE-X] Avoid timeout -> skip WP{waypoint_idx}", flush=True)
            waypoint_idx = (waypoint_idx + 1) % len(WAYPOINTS)
            avoid_timer = 0
            state = FSM.PATROL

    elif state == FSM.INTRUDER_DETECTED:
        # Slow to a crawl and classify for 600 ms; LEDs solid to signal detection.
        drive(PATROL_SPEED * 0.3, PATROL_SPEED * 0.3)
        set_alert_leds(True, "solid")
        scan_timer += TIME_STEP
        if scan_timer > 600:
            scan_timer = 0
            if threat in ("HIGH", "MEDIUM"):
                state = FSM.NAVIGATE_TO_TARGET
            else:
                state = FSM.PATROL
                set_alert_leds(False)

    elif state == FSM.NAVIGATE_TO_TARGET:
        set_alert_leds(True, "blink")
        if vision["intruder_x"] is None:
            state = FSM.SCAN
            scan_timer = 0
        else:
            err = (vision["intruder_x"] - CAM_W / 2) / (CAM_W / 2)
            base = INTERCEPT_SPEED
            drive(base + err * TURN_SPEED, base - err * TURN_SPEED)
            if vision["red_ratio"] >= RED_PIXEL_RATIO_CLOSE or prox["front"] > INTERCEPT_DISTANCE:
                state = FSM.INTERCEPT
                alert_timer = 0

    elif state == FSM.INTERCEPT:
        set_alert_leds(True, "blink")
        if prox["front"] > INTERCEPT_DISTANCE * 1.3:
            drive(-1.0, -1.0)
        elif prox["front"] < INTERCEPT_DISTANCE * 0.6:
            drive(1.5, 1.5)
        else:
            stop()
        alert_timer += TIME_STEP
        if alert_timer > 1500:
            alert_timer = 0
            state = FSM.ALERT

    elif state == FSM.ALERT:
        set_alert_leds(True, "blink")
        stop()
        alert_timer += TIME_STEP

        # Announce once per detection.
        if not alert_announced:
            zone_msg = "inside restricted zone" if in_rz else "near restricted zone"
            print(f"[SCOPE-X] >>> ALERT <<<  Intruder {zone_msg}  threat={threat}",
                  flush=True)
            if speaker is not None:
                try:
                    speaker.speak("Alert. Intruder detected.", 1.0)
                except Exception as e:
                    print(f"[SCOPE-X] Speaker error: {e}", flush=True)
            alert_announced = True

        # Resume patrol when:
        #   • threat dropped to LOW or NONE  (intruder moved away / out of view)
        #   • OR 6-second hard timeout       (robot must not freeze forever)
        if threat in ("NONE", "LOW") or alert_timer > 6000:
            set_alert_leds(False)
            state = FSM.PATROL
            alert_announced = False
            alert_timer = 0
            print("[SCOPE-X] Alert cleared -> resuming patrol", flush=True)

    elif state == FSM.SCAN:
        drive(-TURN_SPEED * 0.7, TURN_SPEED * 0.7)
        scan_timer += TIME_STEP
        if vision["red_ratio"] >= RED_PIXEL_RATIO_DETECT:
            state = FSM.INTRUDER_DETECTED
            scan_timer = 0
        elif scan_timer > 3000:
            scan_timer = 0
            state = FSM.PATROL
            patrol_timer = 0

    else:  # FSM.PATROL  -- waypoint following (Wk06 path planning)
        set_alert_leds(False)
        target_x, target_y = WAYPOINTS[waypoint_idx]
        rx, ry = gps_values[0], gps_values[1]
        dx = target_x - rx
        dy = target_y - ry
        dist = math.hypot(dx, dy)

        if dist < WAYPOINT_TOLERANCE:
            # Waypoint reached -> advance to the next one in the loop
            print(f"[SCOPE-X] WP{waypoint_idx} reached at ({rx:.2f},{ry:.2f}); "
                  f"in_rz={in_rz}", flush=True)
            waypoint_idx = (waypoint_idx + 1) % len(WAYPOINTS)
        else:
            # Heading control: rotate to face the waypoint, then drive forward.
            cur_h = get_heading()
            tgt_h = target_bearing(rx, ry, target_x, target_y)
            herr = angle_diff(tgt_h, cur_h)

            if abs(herr) > HEADING_TOLERANCE:
                # Rotate in place towards target bearing.
                # herr > 0  -> need CCW rotation (turn left): L back, R fwd
                # herr < 0  -> need CW  rotation (turn right): L fwd, R back
                sign = 1.0 if herr > 0 else -1.0
                drive(-sign * TURN_SPEED, sign * TURN_SPEED)
            else:
                # Drive forward, biasing wheels by heading error (smooth curves)
                steer = max(-1.0, min(1.0, herr * HEADING_KP))
                drive(PATROL_SPEED * (1 - steer),
                      PATROL_SPEED * (1 + steer))

    # ----- Stall detection -----
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

"""
SCOPE-X fixed room waypoint controller
======================================

This version fixes the patrol path so room waypoints are clearly inside each room,
not too close to the door/corridor points.

Main waypoint fixes:
- Door points stay near openings.
- Room points are deeper inside rooms and spaced far enough apart.
- Tolerance is reduced so the robot does not skip room points.
- Still avoids extreme wall/corner coordinates.

Use this as:
controllers/scope_controller/scope_controller.py
"""

from controller import Robot
import math

# ============================================================
# SETUP
# ============================================================
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ps = []
for i in range(8):
    s = robot.getDevice(f"ps{i}")
    s.enable(TIME_STEP)
    ps.append(s)

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
CAM_W = camera.getWidth()
CAM_H = camera.getHeight()

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

iu = robot.getDevice("iu")
iu.enable(TIME_STEP)

speaker = None
try:
    speaker = robot.getDevice("speaker")
except Exception:
    speaker = None

leds = []
for i in range(10):
    try:
        led = robot.getDevice(f"led{i}")
        if led:
            leds.append(led)
    except Exception:
        pass

# ============================================================
# CONFIG
# ============================================================
GPS_PLANE = "XY"   # Your arena uses X/Y. Change to "XZ" only if movement is wrong.

MAX_SPEED = 6.28

PATROL_SPEED = 3.20
ROOM_SPEED = 2.0
TURN_SPEED = 1.45
ESCAPE_SPEED = 1.25

# Corridor can avoid earlier. Door/room only avoid when very close.
FRONT_BLOCK = 520.0
EMERGENCY_FRONT = 1650.0

# Reduced tolerance so room points are not skipped.
WAYPOINT_TOL = 0.18
DOOR_TOL = 0.20
ROOM_TOL = 0.18
HEADING_TOL = 0.48

AVOID_TURN_MS = 850
AVOID_ESCAPE_MS = 700

RED_RATIO_ALERT = 0.070
RED_CENTER_LIMIT = 0.55
ALERT_MS = 3500
IGNORE_RED_MS = 7000

# ============================================================
# FIXED ROOM PATROL WAYPOINTS
# ============================================================
# Format: (x, y, mode)
#
# Axis:
# +X = right
# -X = left
# +Y = top
# -Y = bottom
#
# Each room pattern:
# corridor -> door centre -> just inside -> room sweep 1 -> room sweep 2
# -> room sweep 3 -> exit door -> corridor
#
# Values stay around ±0.95 max so the robot avoids walls/corners.
WAYPOINTS = [

    # ---------- START ----------
    (0.00,  0.00, "corridor"),

    # ========================================================
    # TOP LEFT ROOM  (-X, +Y)
    # ========================================================
    (0.00,  0.52, "corridor"),
    (-0.30, 0.56, "door"),
    (-0.48, 0.68, "door"),
    (-0.66, 0.82, "room"),
    (-0.86, 0.98, "room"),
    (-0.84, 1.20, "room"),
    (-1.30, 0.70, "room"),
    (-0.34, 0.62, "door"),
    (0.00,  0.52, "corridor"),

    # ========================================================
    # TOP RIGHT ROOM  (+X, +Y)
    # ========================================================
    (0.60,  0.56, "door"),
    (0.68,  0.68, "door"),
    (0.66, 0.82, "room"),
    (0.86, 0.98, "room"),
    (0.84, 1.20, "room"),
    (1.40, 0.70, "room"),
    (0.34,  0.62, "door"),
    (0.00,  0.52, "corridor"),

    # ========================================================
    # CENTRAL / GREEN BOX CORRIDOR PATROL 
    # ========================================================
    (0.00,  0.00, "corridor"),

    # left green box patrol
    (-1.40, 0.20, "corridor"),

    
    
    
    (0.00,  0.00, "corridor"),

    # left green box patrol
    (1.40, 0.20, "corridor"),

   

    (0.00,  0.00, "corridor"),

    # ========================================================
    # BOTTOM RIGHT ROOM  (+X, -Y)
    # ========================================================
    (0.30, -0.56, "door"),
    (0.48, -0.68, "door"),
    (0.66, -0.82, "room"),
    (0.86, -0.98, "room"),
    (0.84, -1.20, "room"),
    (1.40, -0.70, "room"),
    (0.34, -0.62, "door"),
    (0.00, -0.52, "corridor"),

    # ========================================================
    # BOTTOM LEFT ROOM  (-X, -Y)
    # ========================================================
    (-0.60, -0.56, "door"),
    (-0.68, -0.68, "door"),
    (-0.66, -0.82, "room"),
    (-0.86, -0.98, "room"),
    (-0.84, -1.20, "room"),
    (-1.40, -0.70, "room"),
    (-0.34, -0.62, "door"),
    (0.00, -0.52, "corridor"),

    # ---------- LOOP ----------
    (0.00,  0.00, "corridor"),
]
# ============================================================
# STATES
# ============================================================
PATROL = "PATROL"
AVOID = "AVOID"
ALERT = "ALERT"

state = PATROL
last_state = None

wp = 0
avoid_timer = 0
avoid_dir = 1
ignore_red_timer = 0
alert_timer = 0
alert_said = False

# ============================================================
# HELPERS
# ============================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def drive(l, r):
    left_motor.setVelocity(clamp(l, -MAX_SPEED, MAX_SPEED))
    right_motor.setVelocity(clamp(r, -MAX_SPEED, MAX_SPEED))

def stop():
    drive(0, 0)

def set_leds(on, blink=False):
    for i, led in enumerate(leds):
        if not on:
            led.set(0)
        elif blink:
            led.set(1 if (int(robot.getTime() * 5) + i) % 2 == 0 else 0)
        else:
            led.set(1)

def get_pos():
    v = gps.getValues()
    if GPS_PLANE == "XZ":
        return v[0], v[2]
    return v[0], v[1]

def get_yaw():
    return iu.getRollPitchYaw()[2]

def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def read_ir():
    vals = [s.getValue() for s in ps]
    return {
        "vals": vals,
        "front": max(vals[0], vals[7]),
        "left": max(vals[5], vals[6]),
        "right": max(vals[1], vals[2]),
        "front_left": max(vals[6], vals[7]),
        "front_right": max(vals[0], vals[1]),
        "max": max(vals),
    }

def current_mode():
    return WAYPOINTS[wp][2]

def current_tolerance():
    m = current_mode()
    if m == "door":
        return DOOR_TOL
    if m == "room":
        return ROOM_TOL
    return WAYPOINT_TOL

def current_speed():
    return ROOM_SPEED if current_mode() == "room" else PATROL_SPEED

def choose_avoid_dir(ir):
    if ir["left"] > ir["right"]:
        return 1
    return -1

def turn_away(d):
    if d == 1:
        drive(TURN_SPEED, -TURN_SPEED)
    else:
        drive(-TURN_SPEED, TURN_SPEED)

def escape_away(d):
    if d == 1:
        drive(ESCAPE_SPEED, ESCAPE_SPEED * 0.35)
    else:
        drive(ESCAPE_SPEED * 0.35, ESCAPE_SPEED)

def go_to_waypoint():
    global wp

    rx, ry = get_pos()
    tx, ty, mode = WAYPOINTS[wp]
    d = math.hypot(tx - rx, ty - ry)

    if d < current_tolerance():
        
        wp = (wp + 1) % len(WAYPOINTS)
        return

    target_h = math.atan2(ty - ry, tx - rx)
    err = wrap(target_h - get_yaw())

    if abs(err) > HEADING_TOL:
        if err > 0:
            drive(-TURN_SPEED, TURN_SPEED)
        else:
            drive(TURN_SPEED, -TURN_SPEED)
    else:
        steer = clamp(err * 0.95, -0.28, 0.28)
        base = current_speed()
        drive(base * (1 - steer), base * (1 + steer))

def red_close_and_centered():
    if ignore_red_timer > 0:
        return False

    img = camera.getImage()
    if img is None:
        return False

    red = 0
    red_x = 0
    sampled = 0

    for y in range(0, CAM_H, 3):
        for x in range(0, CAM_W, 3):
            sampled += 1
            r = camera.imageGetRed(img, CAM_W, x, y)
            g = camera.imageGetGreen(img, CAM_W, x, y)
            b = camera.imageGetBlue(img, CAM_W, x, y)

            if r > 210 and g < 70 and b < 70:
                red += 1
                red_x += x

    if sampled == 0 or red == 0:
        return False

    ratio = red / sampled
    centre = red_x / red
    centre_error = abs((centre - CAM_W / 2) / (CAM_W / 2))

    return ratio >= RED_RATIO_ALERT and centre_error <= RED_CENTER_LIMIT

# ============================================================
# MAIN LOOP
# ============================================================


while robot.step(TIME_STEP) != -1:
    ir = read_ir()

    if ignore_red_timer > 0:
        ignore_red_timer -= TIME_STEP

    mode = current_mode()

    # ---------------- TRANSITIONS ----------------
    if state == PATROL:
        if red_close_and_centered():
            state = ALERT
            alert_timer = 0
            alert_said = False
        else:
            # In door/room mode, only avoid emergency front collision.
            # This prevents door frames from blocking room entry.
            if mode in ("door", "room"):
                if ir["front"] > EMERGENCY_FRONT:
                    avoid_dir = choose_avoid_dir(ir)
                    avoid_timer = 0
                    state = AVOID
            else:
                if ir["front"] > FRONT_BLOCK:
                    avoid_dir = choose_avoid_dir(ir)
                    avoid_timer = 0
                    state = AVOID

    if state != last_state:
        x, y = get_pos()
        
        last_state = state

    # ---------------- ACTIONS ----------------
    if state == ALERT:
        stop()
        set_leds(True, blink=True)
        alert_timer += TIME_STEP

        if not alert_said:
            print("[SCOPE-X] ALERT: close red object detected.", flush=True)
            if speaker:
                try:
                    speaker.speak("Alert. Intruder detected.", 1.0)
                except Exception:
                    pass
            alert_said = True

        if alert_timer > ALERT_MS:
            set_leds(False)
            alert_timer = 0
            alert_said = False
            ignore_red_timer = IGNORE_RED_MS
            wp = (wp + 1) % len(WAYPOINTS)
            state = PATROL
            print("[SCOPE-X] Alert finished, resuming patrol.", flush=True)

    elif state == AVOID:
        set_leds(False)
        avoid_timer += TIME_STEP

        if avoid_timer < AVOID_TURN_MS or ir["front"] > EMERGENCY_FRONT:
            turn_away(avoid_dir)
        elif avoid_timer < AVOID_TURN_MS + AVOID_ESCAPE_MS:
            escape_away(avoid_dir)
        else:
            print(f"[SCOPE-X] Avoid completed. Skipping WP{wp}.", flush=True)
            wp = (wp + 1) % len(WAYPOINTS)
            avoid_timer = 0
            state = PATROL

    else:
        set_leds(False)
        go_to_waypoint()

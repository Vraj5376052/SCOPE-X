"""
SCOPE-X — Security Robot Controller  v2
=========================================
New in v2:
  - Zone-aware alarm: alarm ONLY fires when intruder is detected AND robot
    is at a restricted-zone peek waypoint (doorway of TR or BL room).
  - Robot never enters restricted zones.
  - Dwell + scan at each restricted doorway before moving on.
  - Intruder spotted in free zone → silent track only (no alarm).

Map layout (Webots top-down, arena ~3m x 3m centered at 0,0):
        TL-FREE   |  TR-RESTRICTED
        BL-RESTRIC|  BR-FREE

Waypoints (loop, 9 stops):
  1 center          ( 0.00,  0.00)  free
  2 TL room         (-0.90, -0.90)  free  — robot enters
  3 N corridor      ( 0.00, -0.50)  free
  4 TR doorway peek ( 0.50, -0.50)  *** ALARM ZONE ***
  5 E corridor      ( 0.55,  0.00)  free
  6 BR room         ( 0.90,  0.90)  free  — robot enters
  7 S corridor      ( 0.00,  0.50)  free
  8 BL doorway peek (-0.50,  0.50)  *** ALARM ZONE ***
  9 W corridor      (-0.55,  0.00)  free
"""

from controller import Robot
import math

robot    = Robot()
timestep = int(robot.getBasicTimeStep())

# ---------- MOTORS ----------
leftMotor  = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

# ---------- SPEEDS ----------
PATROL_SPD = 3.5
CHASE_SPD  = 5.0
TURN_SPD   = 3.0

# ---------- DISTANCE SENSORS ----------
sensors = []
for i in range(8):
    s = robot.getDevice("ps" + str(i))
    s.enable(timestep)
    sensors.append(s)
OBS_THRESH = 80

# ---------- GPS ----------
gps = robot.getDevice("gps")
gps.enable(timestep)

# ---------- COMPASS ----------
compass = robot.getDevice("compass")
compass.enable(timestep)

# ---------- CAMERA ----------
camera = robot.getDevice("camera")
camera.enable(timestep)
CAM_W  = camera.getWidth()
CAM_H  = camera.getHeight()
CAM_CX = CAM_W / 2.0

# ---------- LED ----------
try:
    led     = robot.getDevice("led0")
    led.set(0)
    HAS_LED = True
except Exception:
    HAS_LED = False

# ---------- SPEAKER ----------
try:
    speaker     = robot.getDevice("speaker")
    HAS_SPEAKER = True
except Exception:
    HAS_SPEAKER = False

# ---------- GREEN DETECTION THRESHOLDS ----------
# Set intruder baseColor to "0 1 0" in Webots
G_MIN       = 150
R_MAX       = 80
B_MAX       = 80
G_DOM       = 80    # G must exceed R and B by this much
MIN_BLOB    = 30
CENTER_DZ   = 0.12  # ±12% of frame width = "centered"
KP_STEER    = 2.5

# ---------- WAYPOINTS (x, z, is_alarm_peek) ----------
# Adjust x/z to match your actual arena dimensions.
WAYPOINTS = [
    ( 0.00,  0.00, False),   # 1 center
    (-0.90, -0.90, False),   # 2 TL room (free — robot enters)
    ( 0.00, -0.50, False),   # 3 N corridor
    ( 0.50, -0.50, True ),   # 4 TR doorway peek  ← ALARM ZONE
    ( 0.55,  0.00, False),   # 5 E corridor
    ( 0.90,  0.90, False),   # 6 BR room (free — robot enters)
    ( 0.00,  0.50, False),   # 7 S corridor
    (-0.50,  0.50, True ),   # 8 BL doorway peek  ← ALARM ZONE
    (-0.55,  0.00, False),   # 9 W corridor
]
ARRIVE_M   = 0.18   # metres — "arrived"
PEEK_MS    = 2000   # dwell time at peek waypoints (ms) — slow pan scan

# ---------- FSM STATES ----------
S_PATROL  = "PATROL"
S_PEEK    = "PEEK"
S_CHASE   = "CHASE"
S_CHARGE  = "CHARGE"
S_LOST    = "LOST"
S_AVD_F   = "AVOID_FRONT"
S_AVD_L   = "AVOID_LEFT"
S_AVD_R   = "AVOID_RIGHT"

state      = S_PATROL
wp_idx     = 0
lost_ms    = 0
peek_ms    = 0
LOST_LIMIT = 1600

# ---------- ALARM ----------
alarm_on    = False
flash_ms    = 0
FLASH_RATE  = 280
led_lit     = False
snd_ms      = 0
SND_EVERY   = 2200


def set_spd(l, r):
    leftMotor.setVelocity(max(-6.28, min(6.28, l)))
    rightMotor.setVelocity(max(-6.28, min(6.28, r)))

def stop():
    set_spd(0, 0)

def get_pos():
    v = gps.getValues()
    return v[0], v[2]

def get_heading():
    v = compass.getValues()
    return math.atan2(v[0], v[2])

def dist_to(wx, wz):
    x, z = get_pos()
    return math.sqrt((wx - x) ** 2 + (wz - z) ** 2)

def heading_diff(wx, wz):
    x, z   = get_pos()
    target = math.atan2(wx - x, wz - z)
    diff   = target - get_heading()
    while diff >  math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return diff

def drive_to(wx, wz, speed=PATROL_SPD):
    if dist_to(wx, wz) < ARRIVE_M:
        return True
    corr = 3.5 * heading_diff(wx, wz)
    set_spd(speed - corr, speed + corr)
    return False

def steer_cam(err, speed=CHASE_SPD):
    c = KP_STEER * err
    set_spd(speed + c, speed - c)

def set_alarm(on):
    global alarm_on, flash_ms, snd_ms, led_lit
    if on == alarm_on:
        return
    alarm_on = on
    flash_ms = 0
    snd_ms   = 0
    led_lit  = False
    if HAS_LED:
        led.set(0)

def tick_alarm():
    global flash_ms, led_lit, snd_ms
    flash_ms += timestep
    snd_ms   += timestep
    if flash_ms >= FLASH_RATE:
        flash_ms = 0
        led_lit  = not led_lit
        if HAS_LED:
            led.set(1 if led_lit else 0)
    if snd_ms >= SND_EVERY:
        snd_ms = 0
        if HAS_SPEAKER:
            try:
                speaker.playSound(speaker, speaker,
                                  "sounds/intruder_alert.wav",
                                  1.0, 1.0, 0, False)
            except Exception:
                pass

def detect_green(img):
    if not img:
        return False, 0.0, 0
    try:
        import numpy as np
        a    = np.frombuffer(img, dtype=np.uint8).reshape((CAM_H, CAM_W, 4))
        b    = a[:, :, 0].astype(np.int16)
        g    = a[:, :, 1].astype(np.int16)
        r    = a[:, :, 2].astype(np.int16)
        mask = ((g >= G_MIN) & (r <= R_MAX) & (b <= B_MAX)
                & ((g - r) > G_DOM) & ((g - b) > G_DOM))
        sz   = int(np.sum(mask))
        if sz < MIN_BLOB:
            return False, 0.0, sz
        return True, float(np.mean(np.where(mask)[1])), sz
    except ImportError:
        hits = []
        for i in range(CAM_W * CAM_H):
            base = i * 4
            bv, gv, rv = img[base], img[base+1], img[base+2]
            if (gv >= G_MIN and rv <= R_MAX and bv <= B_MAX
                    and gv - rv > G_DOM and gv - bv > G_DOM):
                hits.append(i % CAM_W)
        sz = len(hits)
        if sz < MIN_BLOB:
            return False, 0.0, sz
        return True, sum(hits) / sz, sz


# =======================================================================
# MAIN LOOP
# =======================================================================
while robot.step(timestep) != -1:

    sv    = [s.getValue() for s in sensors]
    front = sv[0] > OBS_THRESH or sv[7] > OBS_THRESH
    left  = sv[5] > OBS_THRESH or sv[6] > OBS_THRESH
    right = sv[1] > OBS_THRESH or sv[2] > OBS_THRESH

    img          = camera.getImage()
    found, cx, blob = detect_green(img)
    err          = (cx - CAM_CX) / CAM_CX if found else 0.0

    wx, wz, is_peek = WAYPOINTS[wp_idx]

    # Alarm fires ONLY when intruder seen from an alarm-zone peek waypoint
    at_alarm_post = (state in (S_PEEK, S_CHASE, S_CHARGE, S_LOST)) and is_peek
    should_alarm  = found and at_alarm_post

    if should_alarm:
        set_alarm(True)
        tick_alarm()
    else:
        set_alarm(False)

    # ------------------------------------------------------------------
    # FSM
    # ------------------------------------------------------------------

    if front:
        state = S_AVD_F
        set_spd(-TURN_SPD, TURN_SPD)

    elif left:
        state = S_AVD_L
        set_spd(TURN_SPD, 1.0)

    elif right:
        state = S_AVD_R
        set_spd(1.0, TURN_SPD)

    elif found:
        # Intruder visible — chase regardless of zone,
        # but alarm only fires if at a peek waypoint (handled above)
        lost_ms = 0
        if abs(err) <= CENTER_DZ:
            state = S_CHARGE
            # At peek waypoint: approach to doorway edge but don't cross in
            if is_peek and dist_to(wx, wz) < 0.30:
                stop()
            else:
                steer_cam(0)
        else:
            state = S_CHASE
            steer_cam(err, speed=CHASE_SPD * 0.55)

    elif state in (S_CHASE, S_CHARGE, S_LOST):
        lost_ms += timestep
        if lost_ms < LOST_LIMIT:
            state = S_LOST
            # Slow clockwise scan to reacquire
            set_spd(TURN_SPD * 0.4, -TURN_SPD * 0.4)
        else:
            lost_ms = 0
            state   = S_PATROL
            set_alarm(False)

    else:
        # Patrol — drive to waypoint
        arrived = drive_to(wx, wz)
        if arrived:
            if is_peek:
                # Dwell and scan
                state    = S_PEEK
                peek_ms += timestep
                stop()
                if peek_ms >= PEEK_MS:
                    peek_ms = 0
                    wp_idx  = (wp_idx + 1) % len(WAYPOINTS)
            else:
                state   = S_PATROL
                peek_ms = 0
                wp_idx  = (wp_idx + 1) % len(WAYPOINTS)
        else:
            state = S_PATROL

    rx, rz = get_pos()
    print(
        f"[{state:<12}] wp={wp_idx}|{'ALARM_POST' if is_peek else 'free      '} "
        f"pos=({rx:+.2f},{rz:+.2f}) blob={blob:4d} "
        f"{'!! ALARM !!' if alarm_on else '           '} "
        f"lost={lost_ms:4d}ms",
        flush=True
    )
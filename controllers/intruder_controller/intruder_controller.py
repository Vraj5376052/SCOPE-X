# SCOPE-X — Intruder Controller


from controller import Robot
import math
import random

robot    = Robot()
timestep = int(robot.getBasicTimeStep())

# ---------- MOTORS ----------
leftMotor  = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

# ---------- DISTANCE SENSORS ----------
sensors = []
for i in range(8):
    s = robot.getDevice("ps" + str(i))
    s.enable(timestep)
    sensors.append(s)
OBS_THRESH  = 80
FLEE_THRESH = 150   # high sensor read = robot nearby on that side

# ---------- GPS ----------
gps = robot.getDevice("gps")
gps.enable(timestep)

# ---------- COMPASS ----------
compass = robot.getDevice("compass")
compass.enable(timestep)

# ---------- SPEEDS ----------
MOVE_SPD  = 3.0
FLEE_SPD  = 5.5
TURN_SPD  = 2.5

# ---------- WANDER TARGETS ----------
# Intruder prefers restricted zones (weight 3) over free zones (weight 1).
# Each entry: (x, z, label, weight)
TARGETS = [
    # Restricted zones — intruder's "home turf"
    ( 0.75, -0.75, "TR_deep",   3),   # deep in TR restricted room
    ( 0.60, -0.60, "TR_mid",    3),   # mid TR room
    (-0.75,  0.75, "BL_deep",   3),   # deep in BL restricted room
    (-0.60,  0.60, "BL_mid",    3),   # mid BL room
    # Corridors — transit paths between restricted zones
    ( 0.00,  0.00, "center",    1),
    ( 0.00, -0.35, "N_cdr",     1),
    ( 0.35,  0.00, "E_cdr",     1),
    ( 0.00,  0.35, "S_cdr",     1),
    (-0.35,  0.00, "W_cdr",     1),
    # Free zones — intruder visits rarely
    (-0.75, -0.75, "TL_room",   1),
    ( 0.75,  0.75, "BR_room",   1),
]

# Build weighted target list
WEIGHTED = []
for t in TARGETS:
    WEIGHTED.extend([t] * t[3])

# ---------- STATE ----------
STATE_MOVE  = "MOVE"
STATE_TURN  = "TURN"
STATE_FLEE  = "FLEE"
STATE_AVOID = "AVOID"

state         = STATE_MOVE
target_idx    = 0
current_target = random.choice(WEIGHTED)
turn_dir      = 1   # +1 or -1
turn_ms       = 0
TURN_MAX_MS   = 1200
idle_ms       = 0
IDLE_AT_MS    = 3000   # linger at target before picking next one

ARRIVE_M      = 0.20


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

def drive_to(wx, wz, speed=MOVE_SPD):
    if dist_to(wx, wz) < ARRIVE_M:
        return True
    corr = 3.0 * heading_diff(wx, wz)
    set_spd(speed - corr, speed + corr)
    return False

def pick_new_target():
    return random.choice(WEIGHTED)


# =======================================================================
# MAIN LOOP
# =======================================================================
while robot.step(timestep) != -1:

    sv     = [s.getValue() for s in sensors]
    front  = sv[0] > OBS_THRESH or sv[7] > OBS_THRESH
    left   = sv[5] > OBS_THRESH or sv[6] > OBS_THRESH
    right  = sv[1] > OBS_THRESH or sv[2] > OBS_THRESH

    # Detect if robot is nearby (any sensor very high = close obstacle)
    max_sv = max(sv)
    robot_close = max_sv > FLEE_THRESH

    # ------------------------------------------------------------------
    # FSM
    # ------------------------------------------------------------------

    if robot_close:
        # FLEE — spin away from the highest sensor reading
        state = STATE_FLEE
        # Find which side the threat is on
        threat_left  = max(sv[5], sv[6]) > max(sv[1], sv[2])
        if threat_left:
            set_spd(FLEE_SPD, -FLEE_SPD * 0.3)   # veer right
        else:
            set_spd(-FLEE_SPD * 0.3, FLEE_SPD)   # veer left

    elif front:
        state   = STATE_AVOID
        turn_dir = random.choice([-1, 1])
        set_spd(turn_dir * TURN_SPD, -turn_dir * TURN_SPD)

    elif left:
        state = STATE_AVOID
        set_spd(TURN_SPD, 0.5)

    elif right:
        state = STATE_AVOID
        set_spd(0.5, TURN_SPD)

    else:
        # Navigate to current target
        tx, tz, tlabel, _ = current_target
        arrived = drive_to(tx, tz)
        if arrived:
            state    = STATE_MOVE
            idle_ms += timestep
            stop()
            # Linger, then pick a new target
            if idle_ms >= IDLE_AT_MS:
                idle_ms        = 0
                current_target = pick_new_target()
        else:
            state   = STATE_MOVE
            idle_ms = 0

    rx, rz = get_pos()
    tx, tz, tlabel, _ = current_target
    print(
        f"[INTRUDER {state:<8}] pos=({rx:+.2f},{rz:+.2f}) "
        f"→ target={tlabel} dist={dist_to(tx, tz):.2f}m "
        f"flee={'YES' if robot_close else 'no'}",
        flush=True
    )
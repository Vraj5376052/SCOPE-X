"""
SCOPE-X — Security Robot Controller  v4
=========================================
Patrol route: perimeter-hugging loop through all free zones,
peeking at each restricted doorway. Matches the red line drawn on map.

Route (continuous loop):
  BR free room  → right wall hug → up right corridor
  → TR doorway PEEK (alarm zone)
  → across top corridor → TL free room perimeter sweep
  → down left corridor → BL doorway PEEK (alarm zone)
  → down bottom corridor → BR free room perimeter sweep → loop

No GPS/Compass needed. Timer-based steps.
Each step: (left_spd, right_spd, duration_ms, is_restricted_peek)

Tuning tip: 700ms ≈ 90° turn at TURN_SPD=2.5, PATROL_SPD=3.5
            Adjust durations if robot drifts off route.
"""

from controller import Robot

robot    = Robot()
timestep = int(robot.getBasicTimeStep())

# ---------- MOTORS ----------
leftMotor  = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

# ---------- SPEEDS ----------
P  = 3.5   # patrol forward
T  = 2.5   # turn speed
C  = 5.0   # chase speed

# Derived turn combos
CW  = ( T, -T)   # clockwise (turn right)
CCW = (-T,  T)   # counter-clockwise (turn left)
FWD = ( P,  P)   # forward
STP = ( 0,  0)   # stop

# ---------- DISTANCE SENSORS ----------
sensors = []
for i in range(8):
    s = robot.getDevice("ps" + str(i))
    s.enable(timestep)
    sensors.append(s)
OBS = 80

# ---------- CAMERA ----------
camera = robot.getDevice("camera")
camera.enable(timestep)
W  = camera.getWidth()
H  = camera.getHeight()
CX = W / 2.0

# ---------- LEDs ----------
leds = []
for i in range(8):
    try:
        leds.append(robot.getDevice(f"led{i}"))
    except Exception:
        pass

def set_leds(v):
    for l in leds:
        try: l.set(v)
        except: pass

# ---------- SPEAKER ----------
try:
    speaker = robot.getDevice("speaker")
    HAS_SND = True
except Exception:
    HAS_SND = False

# ---------- DETECTION ----------
G_MIN = 150; R_MAX = 80; B_MAX = 80; G_DOM = 80
MIN_B = 30;  DZ = 0.12;  KP = 2.5

def detect_green(img):
    if not img: return False, 0.0, 0
    try:
        import numpy as np
        a = np.frombuffer(img, dtype=np.uint8).reshape((H, W, 4))
        b = a[:,:,0].astype('int16'); g = a[:,:,1].astype('int16'); r = a[:,:,2].astype('int16')
        mask = (g>=G_MIN)&(r<=R_MAX)&(b<=B_MAX)&((g-r)>G_DOM)&((g-b)>G_DOM)
        sz = int(mask.sum())
        if sz < MIN_B: return False, 0.0, sz
        return True, float(np.where(mask)[1].mean()), sz
    except ImportError:
        hits = [i % W for i in range(W*H)
                if img[i*4+1]>=G_MIN and img[i*4+2]<=R_MAX and img[i*4]<=B_MAX
                and img[i*4+1]-img[i*4+2]>G_DOM and img[i*4+1]-img[i*4]>G_DOM]
        sz = len(hits)
        if sz < MIN_B: return False, 0.0, sz
        return True, sum(hits)/sz, sz

# PATROL SEQUENCE
PATROL_SEQ = [
    # ── BR free room: hug bottom wall ────────────────────────────────
    (*FWD, 20000, False),   # forward along bottom wall of BR room
    (*CW,    600, False),   # turn right 90°
    (*FWD, 12000, False),   # forward along right wall of BR room
    (*CW,    600, False),   # turn right 90° → now facing north (up)

    # ── Right corridor: head north toward TR doorway ──────────────────
    (*FWD, 20000, False),   # forward up right corridor
    (*CCW,   600, False),   # turn left 90° → face TR doorway

    # ── TR restricted doorway PEEK ────────────────────────────────────
    (*STP,  3000, True ),   # stop and scan
    (*CW,    900, True ),   # pan right
    (*CCW,  1800, True ),   # pan left across doorway
    (*CW,    900, True ),   # re-center
    (*STP,   800, True ),   # final hold

    # ── Top corridor: cross to TL side ───────────────────────────────
    (*CCW,   600, False),   # turn left 90° → face west
    (*FWD, 28000, False),   # forward across full top corridor

    # ── TL free room: perimeter sweep ────────────────────────────────
    (*CCW,   600, False),   # turn left 90° → face top wall
    (*FWD, 12000, False),   # along top wall
    (*CCW,   600, False),   # turn left 90° → face left wall
    (*FWD, 20000, False),   # along left wall
    (*CCW,   600, False),   # turn left 90° → face bottom of TL room
    (*FWD, 10000, False),   # toward divider wall
    (*CW,    600, False),   # turn right 90° → face back to corridor

    # ── Left corridor: head south toward BL doorway ──────────────────
    (*FWD, 20000, False),   # forward down left corridor
    (*CW,    600, False),   # turn right 90° → face BL doorway

    # ── BL restricted doorway PEEK ───────────────────────────────────
    (*STP,  3000, True ),   # stop and scan
    (*CCW,   900, True ),   # pan left
    (*CW,   1800, True ),   # pan right across doorway
    (*CCW,   900, True ),   # re-center
    (*STP,   800, True ),   # final hold

    # ── Bottom corridor: cross back to BR side ────────────────────────
    (*CW,    600, False),   # turn right 90° → face east
    (*FWD, 28000, False),   # forward across full bottom corridor

    # ── BR free room: re-entry sweep ─────────────────────────────────
    (*CW,    600, False),   # turn right 90° → face bottom wall
    (*FWD, 10000, False),   # back along bottom wall → restart loop
]

# ---------- FSM ----------
S_PATROL = "PATROL"
S_PEEK   = "PEEK"
S_CHASE  = "CHASE"
S_CHARGE = "CHARGE"
S_LOST   = "LOST"
S_AVD_F  = "AVOID_F"
S_AVD_L  = "AVOID_L"
S_AVD_R  = "AVOID_R"

state   = S_PATROL
si      = 0          # sequence index
sel     = 0          # sequence elapsed ms
lost_ms = 0
LOST_LIM = 1600

# ---------- ALARM ----------
alarm_on  = False
flash_ms  = 0
FLASH     = 280
led_lit   = False
snd_ms    = 0
SND_INT   = 2200

def set_spd(l, r):
    leftMotor.setVelocity(max(-6.28, min(6.28, l)))
    rightMotor.setVelocity(max(-6.28, min(6.28, r)))

def steer_cam(err, spd=C):
    c = KP * err
    set_spd(spd + c, spd - c)

def set_alarm(on):
    global alarm_on, flash_ms, snd_ms, led_lit
    if on == alarm_on: return
    alarm_on = on; flash_ms = 0; snd_ms = 0; led_lit = False
    set_leds(0)

def tick_alarm():
    global flash_ms, led_lit, snd_ms
    flash_ms += timestep; snd_ms += timestep
    if flash_ms >= FLASH:
        flash_ms = 0; led_lit = not led_lit
        set_leds(1 if led_lit else 0)
    if snd_ms >= SND_INT:
        snd_ms = 0
        if HAS_SND:
            try: speaker.playSound(speaker, speaker, "sounds/intruder_alert.wav", 1.0, 1.0, 0, False)
            except: pass

# =======================================================================
# MAIN LOOP
# =======================================================================
while robot.step(timestep) != -1:

    sv    = [s.getValue() for s in sensors]
    front = sv[0] > OBS or sv[7] > OBS
    left  = sv[5] > OBS or sv[6] > OBS
    right = sv[1] > OBS or sv[2] > OBS

    img             = camera.getImage()
    found, cx, blob = detect_green(img)
    err             = (cx - CX) / CX if found else 0.0

    pl, pr, pdur, is_peek = PATROL_SEQ[si]

    should_alarm = found and is_peek and state in (S_PEEK, S_CHASE, S_CHARGE, S_LOST)
    if should_alarm: set_alarm(True);  tick_alarm()
    else:            set_alarm(False)

    # ── FSM ────────────────────────────────────────────────────────────
    if front:
        state = S_AVD_F;  set_spd(-T, T)

    elif left:
        state = S_AVD_L;  set_spd(T, 1.0)

    elif right:
        state = S_AVD_R;  set_spd(1.0, T)

    elif found:
        lost_ms = 0
        if abs(err) <= DZ:
            state = S_CHARGE
            if is_peek: set_spd(0, 0)        # hold at doorway, don't cross
            else:       steer_cam(0)
        else:
            state = S_CHASE
            steer_cam(err, spd=C * 0.55)

    elif state in (S_CHASE, S_CHARGE, S_LOST):
        lost_ms += timestep
        if lost_ms < LOST_LIM:
            state = S_LOST
            set_spd(T * 0.4, -T * 0.4)      # slow scan to reacquire
        else:
            lost_ms = 0; state = S_PATROL; set_alarm(False)

    else:
        # Execute patrol step
        state = S_PEEK if is_peek else S_PATROL
        sel  += timestep
        set_spd(pl, pr)
        if sel >= pdur:
            sel = 0
            si  = (si + 1) % len(PATROL_SEQ)

    print(
        f"[{state:<8}] step={si:02d}/{'PK' if is_peek else '--'} "
        f"t={sel:4d}/{pdur}ms blob={blob:4d} "
        f"{'!! ALARM !!' if alarm_on else '           '}",
        flush=True
    )
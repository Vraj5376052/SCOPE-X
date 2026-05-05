from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ---------- MOTORS ----------
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

# ---------- SPEEDS ----------
FORWARD_SPEED = 4.0
TURN_SPEED = 3.0

# ---------- DISTANCE SENSORS ----------
sensors = []
for i in range(8):
    sensor = robot.getDevice("ps" + str(i))
    sensor.enable(timestep)
    sensors.append(sensor)

# ---------- PATROL TIMER ----------
turn_timer = 0

# ---------- MOVEMENT FUNCTIONS ----------
def set_speed(left, right):
    leftMotor.setVelocity(left)
    rightMotor.setVelocity(right)

def move_forward():
    set_speed(FORWARD_SPEED, FORWARD_SPEED)

def avoid_front():
    set_speed(-TURN_SPEED, TURN_SPEED)

def avoid_left():
    set_speed(TURN_SPEED, 1.0)

def avoid_right():
    set_speed(1.0, TURN_SPEED)

def patrol_turn():
    set_speed(3.0, 2.0)

# ---------- MAIN LOOP ----------
while robot.step(timestep) != -1:

    values = [sensor.getValue() for sensor in sensors]

    front = values[0] > 80 or values[7] > 80
    left = values[5] > 80 or values[6] > 80
    right = values[1] > 80 or values[2] > 80

    # 1. Obstacle avoidance has priority
    if front:
        print("STATE: AVOID_FRONT", flush=True)
        avoid_front()

    elif left:
        print("STATE: AVOID_LEFT", flush=True)
        avoid_left()

    elif right:
        print("STATE: AVOID_RIGHT", flush=True)
        avoid_right()

    # 2. Normal patrol
    else:
        turn_timer += timestep

        if turn_timer < 7000:
            print("STATE: PATROL_FORWARD", flush=True)
            move_forward()

        elif turn_timer < 7600:
            print("STATE: PATROL_TURN", flush=True)
            patrol_turn()

        else:
            turn_timer = 0
            print("STATE: PATROL_RESET", flush=True)
            move_forward()
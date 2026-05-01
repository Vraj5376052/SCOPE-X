from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ---------- MOTORS ----------
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# ---------- SETTINGS ----------
FORWARD_SPEED = 5.5
TURN_SPEED = 4.0

# ---------- MOVEMENT FUNCTIONS ----------
def move_forward():
    leftMotor.setVelocity(FORWARD_SPEED)
    rightMotor.setVelocity(FORWARD_SPEED)

def move_backward():
    leftMotor.setVelocity(-FORWARD_SPEED)
    rightMotor.setVelocity(-FORWARD_SPEED)

def turn_left():
    leftMotor.setVelocity(-TURN_SPEED)
    rightMotor.setVelocity(TURN_SPEED)

def turn_right():
    leftMotor.setVelocity(TURN_SPEED)
    rightMotor.setVelocity(-TURN_SPEED)

def stop_robot():
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

# ---------- PATROL ROUTE ----------
# This is a timed patrol route.
# Tune duration values if robot over/under-shoots.

patrol_steps = [
    ("forward", 180, "Moving through central corridor"),
    ("left", 62, "Turning toward top-left room"),
    ("forward", 90, "Checking top-left room"),
    ("right", 62, "Returning toward corridor"),
    ("forward", 100, "Returning to centre"),

    ("right", 62, "Turning toward top-right restricted zone"),
    ("forward", 110, "Checking top-right restricted zone"),
    ("left", 62, "Turning back to corridor"),
    ("forward", 120, "Returning to centre"),

    ("right", 125, "Turning toward bottom-left restricted zone"),
    ("forward", 120, "Checking bottom-left restricted zone"),
    ("left", 125, "Turning back to centre"),
    ("forward", 100, "Returning to centre"),

    ("left", 62, "Turning toward bottom-right room"),
    ("forward", 100, "Checking bottom-right room"),
    ("right", 62, "Returning to patrol loop"),

    ("stop", 30, "Brief scan pause")
]

current_step = 0
step_counter = 0

# ---------- MAIN LOOP ----------
while robot.step(timestep) != -1:
    action, duration, message = patrol_steps[current_step]

    if step_counter == 0:
        print("PATROL:", message, flush=True)

    if action == "forward":
        move_forward()
    elif action == "backward":
        move_backward()
    elif action == "left":
        turn_left()
    elif action == "right":
        turn_right()
    elif action == "stop":
        stop_robot()

    step_counter += 1

    if step_counter >= duration:
        step_counter = 0
        current_step += 1

        if current_step >= len(patrol_steps):
            current_step = 0
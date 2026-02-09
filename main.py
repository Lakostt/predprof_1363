#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Color, Button
from pybricks.tools import wait, multitask, run_task

# -------------------- initializing --------------------
ev3 = EV3Brick()

right_motor = Motor(Port.D, Direction.CLOCKWISE)
left_motor  = Motor(Port.A, Direction.CLOCKWISE)

track = 10.5
diameter = 4.25

gripper_motor = Motor(
    Port.C,
    Direction.CLOCKWISE,
    gears=[[8, 8], [36, 12], [8, 24]]
)

surface_sensor  = ColorSensor(Port.S1)
cylinder_sensor = ColorSensor(Port.S2)

# -------------------- colors --------------------
road_colors = [Color.BLACK, Color.WHITE, Color.NONE]
surface_colors = [Color.BLACK, Color.WHITE, Color.NONE, Color.YELLOW, Color.RED, Color.GREEN, Color.BLUE]
cylinder_colors = [Color.RED, Color.GREEN, Color.BLUE, Color.WHITE]

surface_sensor.detectable_colors(surface_colors)
cylinder_sensor.detectable_colors(cylinder_colors)

# -------------------- constants --------------------
default_speed = 250
gripper_speed = 750

GRIP_OPEN  = 90
GRIP_CLOSE = 0

# -------------------- helpers --------------------
def wait_for(function, value, coincidence=True, delay=10):
    while (function() == value) != coincidence:
        wait(delay)

def detect_surface_color():
    c = surface_sensor.color()
    return c if c is not None else Color.NONE

def detect_cylinder():
    c = cylinder_sensor.color()
    return c if c is not None else Color.NONE

last_found = "left"

def search_on_left(allowed_angle):
    left_motor.run_angle(default_speed // 2, -allowed_angle * 2 / 3 * track / diameter, wait=False)
    right_motor.run_angle(default_speed,      allowed_angle * 4 / 3 * track / diameter, wait=False)

    while (not left_motor.done() or not right_motor.done()) and (detect_surface_color() != Color.BLACK):
        wait(10)

    if detect_surface_color() == Color.BLACK:
        return True

    left_motor.run_angle(default_speed // 2,  allowed_angle * 2 / 3 * track / diameter, wait=False)
    right_motor.run_angle(default_speed,     -allowed_angle * 4 / 3 * track / diameter, wait=False)

    while not left_motor.done() or not right_motor.done():
        wait(10)

    return False

def search_on_right(allowed_angle):
    left_motor.run_angle(default_speed // 2,  allowed_angle * 2 / 3 * track / diameter, wait=False)
    right_motor.run_angle(default_speed,     -allowed_angle * 4 / 3 * track / diameter, wait=False)

    while (not left_motor.done() or not right_motor.done()) and (detect_surface_color() != Color.BLACK):
        wait(10)

    if detect_surface_color() == Color.BLACK:
        return True

    left_motor.run_angle(default_speed // 2, -allowed_angle * 2 / 3 * track / diameter, wait=False)
    right_motor.run_angle(default_speed,      allowed_angle * 4 / 3 * track / diameter, wait=False)

    while not left_motor.done() or not right_motor.done():
        wait(10)

    return False

def correct_direction(allowed_angle):
    global last_found

    if last_found == "left":
        if not search_on_left(allowed_angle):
            if search_on_right(allowed_angle):
                last_found = "right"
    else:
        if not search_on_right(allowed_angle):
            if search_on_left(allowed_angle):
                last_found = "left"

def go_to_heckpoint():
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    while detect_surface_color() in road_colors:
        if detect_surface_color() == Color.NONE:
            correct_direction(15)

        correct_direction(55)

        right_motor.run(default_speed)
        left_motor.run(default_speed)

        wait_for(detect_surface_color, Color.BLACK, coincidence=False)

        right_motor.hold()
        left_motor.hold()

def go_to_heckpoint_without_correction():
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    right_motor.run(default_speed)
    left_motor.run(default_speed)

    while detect_surface_color() in road_colors:
        wait(10)

    right_motor.hold()
    left_motor.hold()

def go_back():
    left_motor.run_target(default_speed, 0, then=Stop.HOLD, wait=False)
    right_motor.run_target(default_speed, 0, then=Stop.HOLD, wait=False)
    while not left_motor.done() or not right_motor.done():
        wait(10)

async def grab():
    gripper_motor.run_target(gripper_speed, GRIP_OPEN, then=Stop.HOLD, wait=True)

    while True:

        wait_for(detect_cylinder, Color.NONE, coincidence=False)

        gripper_motor.run_target(gripper_speed, GRIP_CLOSE, then=Stop.HOLD, wait=True)

  
        wait_for(detect_cylinder, Color.NONE, coincidence=True)

        gripper_motor.run_target(gripper_speed, GRIP_OPEN, then=Stop.HOLD, wait=True)

        wait(20)


angle = 0

def angle_rotation(rotation_angle):
    global angle

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    right_motor.run_angle(default_speed, -rotation_angle * track / diameter, wait=False)
    left_motor.run_angle(default_speed,   rotation_angle * track / diameter, wait=False)

    while not left_motor.done() or not right_motor.done():
        wait(10)

    angle += rotation_angle

def target_rotation(target):
    global angle
    angle = angle % 360

    if target < 0:
        target += 360

    rotation_angle = target - angle
    if rotation_angle > 180:
        rotation_angle -= 360
    elif rotation_angle < -180:
        rotation_angle += 360

    angle_rotation(rotation_angle)


hatch_angle = {Color.WHITE: 0}

async def main():
    global angle
    angle = 0


    for i in [0, 90, -90]:
        target_rotation(i)
        go_to_heckpoint_without_correction()
        hatch_angle[detect_surface_color()] = i
        go_back()

    for i in [-45, 45, 135, -135]:
        target_rotation(i)
        go_to_heckpoint()
        angle_rotation(180)
        go_to_heckpoint()

        c = detect_cylinder()
        if c != Color.NONE:
            target_rotation(hatch_angle.get(c, hatch_angle[Color.WHITE]))
            go_to_heckpoint_without_correction()
            gripper_motor.run_target(gripper_speed, GRIP_OPEN, then=Stop.HOLD, wait=True)
            go_back()

    target_rotation(-135)
    go_to_heckpoint()
    angle_rotation(45)

    for i in [-45, 45, 135]:
        go_to_heckpoint()
        angle_rotation(45)
        go_to_heckpoint()
        angle += 180

        c = detect_cylinder()
        if c != Color.NONE:
            angle_rotation(45)
            go_to_heckpoint()
            target_rotation(hatch_angle.get(c, hatch_angle[Color.WHITE]))
            go_to_heckpoint_without_correction()
            gripper_motor.run_target(gripper_speed, GRIP_OPEN, then=Stop.HOLD, wait=True)
            go_back()

            target_rotation(i)
            go_to_heckpoint()
            angle_rotation(45)
        else:
            angle_rotation(-45)

    go_to_heckpoint()
    angle_rotation(45)
    go_to_heckpoint()
    angle += 180

    c = detect_cylinder()
    if c != Color.NONE:
        angle_rotation(45)
        go_to_heckpoint()
        target_rotation(hatch_angle.get(c, hatch_angle[Color.WHITE]))
        go_to_heckpoint_without_correction()
        gripper_motor.run_target(gripper_speed, GRIP_OPEN, then=Stop.HOLD, wait=True)

# -------------------- start --------------------
ev3.speaker.beep()


run_task(multitask(main(), grab(), race=True))

# Disha Math 730572610
# Luca Just 730665722

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Direction, Button)
from pybricks.tools import wait, StopWatch
import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.C
US_MOTOR_PORT = Port.B
LEFT_BUMP_PORT = Port.S1
RIGHT_BUMP_PORT = Port.S3
US_PORT = Port.S4

# Robot constants
WHEEL_DIAMETER_METERS = .055
WHEEL_ADJUSTMENT = 1.00
CIRCUMFERENCE = (WHEEL_DIAMETER_METERS * math.pi) * WHEEL_ADJUSTMENT 

SPEED_WALL = 180
KP_WALL = 0.2
KD_WALL = 0.2

TARGET_DISTANCE_WALL = 2.2
TARGET_DISTANCE_MARGIN = 0.05

TARGET_OFFSET_WALL = .10

US_OFFSET_90 = 0
US_BACKUP_DISTANCE = .15
ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT, positive_direction = Direction.CLOCKWISE)
right_motor = Motor(RIGHT_MOTOR_PORT, positive_direction = Direction.CLOCKWISE)
us_motor = Motor(US_MOTOR_PORT, positive_direction = Direction.COUNTERCLOCKWISE)
left_bump_sensor = TouchSensor(LEFT_BUMP_PORT)
right_bump_sensor = TouchSensor(RIGHT_BUMP_PORT)
us_sensor = UltrasonicSensor(US_PORT)

current_objective = 0
current_sw = StopWatch()
current_sw.pause()
current_sw.reset()

def wait_until_center_button():
    current_sw.pause()
    ev3.screen.clear()
    ev3.screen.print("Stopwatch: " + str(current_sw.time() / 1_000) + "s")
    ev3.screen.print("Mission " + str(current_objective))
    ev3.screen.print("Biscuit is ready")
    ev3.screen.print("Heading: " + str(gyroscope.angle()) + "deg")
    while Button.CENTER not in ev3.buttons.pressed():
        wait(50)
    current_sw.resume()

def get_us_distance():
    pass

def rotate_us_sensor_left():
    # resest value before turning
    pass

def get_distance_travelled():
    pass

def reset_distance_travelled():
    pass

def turn_right():
    pass

# run motors with speed adjusted by a (proportion of speed)
# with a > 0, the left motor will spin at a lower speed, and right higher
# When going forward, a = 0 goes straight, a > 0 turns left, a < 0 turns right
def run_motors_proportional(speed = SPEED_WALL, a = 0)
    left_motor.run(speed * (1 - a))
    right_motor.run(speed * (1 + a))

def stop_motors():
    left_motor.brake()
    right_motor.brake()

# follow along a wall on the left for distance meters, and then stop
def follow_wall(distance):
    # assume robot starts with heading parallel to wall
    reset_distance_travelled()
    # go forward
    run_motors_proportional()
    # check odometry distance
    while get_distance_travelled() < distance:
        # check US distance
        e = get_us_distance() - TARGET_OFFSET_WALL
        de = 0 # TODO: calculate rate of change of e?
        # if US distance < target offset, turn left more
        # if US distance > target offset, turn right more
        run_motors_proportional(a = e * KP_WALL + de * KD_WALL)
        # if left bump sensor is hit, make a big rightward correction
        if (left_bump_sensor.pressed()):
            run_motors_proportional(a = -0.5)
        # wait a bit, then repeat from after go forward
        wait(50)
    # if odometry >= target distance, stop and beep
    stop_motors()
    ev3.beep()

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
KP_WALL = 0.2
KD_WALL = 0.2

SPEED_FORWARD = 190
SPEED_BACKWARD = 180

TARGET_DISTANCE_WALL = 2.2
TARGET_DISTANCE_MARGIN = 0.05

TARGET_OFFSET_WALL = .10

US_OFFSET_90 = 0
US_BACKUP_DISTANCE = .10

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
    return us_sensor.distance()/100

def rotate_us_sensor(angle):
    us_motor.reset_angle(angle) # encoder reset         
    us_motor.run_target(100, angle) # should move to angle       
    wait(100)                        

def get_distance_travelled():
    pass

def reset_distance_travelled():
    pass

def turn_right():
    pass

def follow_wall(distance):
    pass

def run_motors_proportional(speed = SPEED_WALL, a = 0)
    left_motor.run(speed * (1 - a))
    right_motor.run(speed * (1 + a))

def stop_motors():
    left_motor.brake()
    right_motor.brake()

def drive_forward_until_bump():
    run_motors_proportional(speed=SPEED_FORWARD, a=0)
    while not right_bump.pressed():
        wait(20)
    stop_motors()
    wait(100)

def backup_for_turn():
    run_motors_proportional(speed=SPEED_BACKWARD, a=0)
    distance = get_us_distance()
    while distance < US_BACKUP_DISTANCE:
        # update distance from ultrasonic
        distance = get_us_distance()
        wait(20)
    stop_motors()


wait_until_center_button()
rotate_us_sensor(0)
drive_forward_until_bump()
backup_for_turn()
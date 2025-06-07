from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# ✅ Setup: Hub, Motors, Sensor
hub = PrimeHub()
left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)  # Left motor on Port E
right_motor = Motor(Port.A)                             # Right motor on Port A
color_sensor = ColorSensor(Port.D)                      # Color sensor on Port D

# 🚗 Function: Drive Straight with Gyro Correction
def move_straight_with_gyro(duration_ms=3000, base_speed=200, kp=2.0):
    hub.imu.reset_heading(0)
    time_elapsed = 0
    while time_elapsed < duration_ms:
        angle = hub.imu.heading()
        correction = angle * kp
        left_motor.run(base_speed - correction)
        right_motor.run(base_speed + correction)
        wait(10)
        time_elapsed += 10
    left_motor.stop()
    right_motor.stop()

# 🔁 Function: Turn Right using Gyro
def turn_to_angle(target_angle, speed=100):
    hub.imu.reset_heading(0)
    while hub.imu.heading() < target_angle:
        left_motor.run(speed)
        right_motor.run(-speed)
        wait(10)
    left_motor.stop()
    right_motor.stop()

# ↩️ Function: Turn Left using Gyro
def turn_left_to_angle(target_angle, speed=100):
    hub.imu.reset_heading(0)
    while hub.imu.heading() > target_angle:
        left_motor.run(-speed)
        right_motor.run(speed)
        wait(10)
    left_motor.stop()
    right_motor.stop()

# 🧠 Function: Line Follow (by degrees, PD control)
def line_follow_degrees(degrees=720, base_speed=150, kp=0.8, kd=2.0):
    left_motor.reset_angle(0)
    last_error = 0
    target_reflection = 50  # Adjust for your surface and line

    while abs(left_motor.angle()) < degrees:
        reflection = color_sensor.reflection()
        error = target_reflection - reflection
        derivative = error - last_error
        correction = kp * error + kd * derivative

        left_motor.run(base_speed - correction)
        right_motor.run(base_speed + correction)

        last_error = error
        wait(10)

    left_motor.stop()
    right_motor.stop()

# 🧪 Example Runs (Uncomment one at a time to test)
# move_straight_with_gyro()
# turn_to_angle(90)
# turn_left_to_angle(-90)
# line_follow_degrees(720)

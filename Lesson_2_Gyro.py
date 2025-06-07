# -------------------------------------------------
# 🎯 GOAL: Learn how to use the Gyro Sensor
# -------------------------------------------------
# In this program, we'll:
# ✅ Set up the motors and hub
# ✅ Use the gyro to turn right and left
# ✅ Drive straight using gyro correction

# -------------------------------------------------
# 🔧 1. Import the necessary libraries
# -------------------------------------------------

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait

# -------------------------------------------------
# ⚙️ 2. Setup: Hub and Motor Configuration
# -------------------------------------------------

hub = PrimeHub()
left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)

# -------------------------------------------------
# 🔄 3. Function: Turn Right using Gyro
# -------------------------------------------------

def turn_to_angle(target_angle, speed=100):
    # Reset the heading to 0 degrees
    hub.imu.reset_heading(0)

    # Get the current heading
    current_angle = hub.imu.heading()

    # Continue turning until the desired angle is reached
    while current_angle < target_angle:
        left_motor.run(speed)
        right_motor.run(-speed)
        current_angle = hub.imu.heading()

    # Stop the motors
    left_motor.stop()
    right_motor.stop()

# 👉 Example: Turn 90 degrees to the right
turn_to_angle(180)

# -------------------------------------------------
# ↩️ 4. Function: Turn Left using Gyro
# -------------------------------------------------

def turn_left_to_angle(target_angle, speed=100):
    # Reset the heading to 0 degrees
    hub.imu.reset_heading(0)

    # Get the current heading
    current_angle = hub.imu.heading()

    # Continue turning until the desired angle is reached
    while current_angle > target_angle:
        left_motor.run(-speed)
        right_motor.run(speed)
        current_angle = hub.imu.heading()

    # Stop the motors
    left_motor.stop()
    right_motor.stop()

# 👉 Example: Turn 90 degrees to the left
turn_left_to_angle(-90)

# -------------------------------------------------
# 🛣️ 5. Function: Drive Straight using Gyro Correction
# -------------------------------------------------

def move_straight_with_gyro(duration_ms=3000, base_speed=200, kp=2):
    # Reset the heading to 0 degrees
    hub.imu.reset_heading(0)

    # Initialize the timer
    time_elapsed = 0

    # Drive straight with gyro correction
    while time_elapsed < duration_ms:
        # Get the current heading
        angle = hub.imu.heading()

        # Calculate the correction
        correction = angle * kp

        # Adjust motor speeds based on correction
        left_motor.run(base_speed - correction)
        right_motor.run(base_speed + correction)

        # Wait for 10 milliseconds
        wait(10)
        time_elapsed += 10

    # Stop the motors
    left_motor.stop()
    right_motor.stop()

# 👉 Example: Drive forward straight for 3 seconds using gyro
move_straight_with_gyro()

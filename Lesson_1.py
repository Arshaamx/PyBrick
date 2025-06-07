# --------------------------------------------
#  Setup: Import Required Libraries & Motors
# --------------------------------------------

# Import the Hub (the brain of the robot)
from pybricks.hubs import PrimeHub

# Import the Motor class so we can control motors
from pybricks.pupdevices import Motor

# Import Port names (A, B, C, etc.) to tell the code which ports to use
from pybricks.parameters import Port, Direction, Stop


# Create an object for the Hub so we can access its features
hub = PrimeHub()

# Create a motor on Port A (usually left motor)
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)

# Create a motor on Port B (usually right motor)
right_motor = Motor(Port.F)

# ----------------------------------------------------
#  Move Forward Using Encoders: 1 Full Wheel Rotation
# ----------------------------------------------------

# Tell each motor to run at 500 degrees per second speed,
# and turn exactly 360 degrees (1 full rotation)
left_motor.run_angle(3000, 3000, then=Stop.HOLD, wait=False)
right_motor.run_angle(3000, 3000)


# -------------------------------
#  Tank Turn: Spin In Place
# -------------------------------

# Left motor moves forward, right motor moves backward
# When both motors move in opposite directions, the robot spins on the spot
left_motor.run_angle(500, -230, then=Stop.HOLD, wait=False)
right_motor.run_angle(500, 230)

left_motor.run_angle(500, 1100, then=Stop.HOLD, wait=False)
right_motor.run_angle(500, 1100)

left_motor.run_angle(500, -230, then=Stop.HOLD, wait=False)
right_motor.run_angle(500, 230)

left_motor.run_angle(5000, 3000, then=Stop.HOLD, wait=False)
right_motor.run_angle(5000, 3000)
# Set the motor's encoder value back to 0
left_motor.reset_angle(0)
right_motor.reset_angle(0)

# Print out the current angle of each motor
# This helps us track how far each motor has turned
print("Left Motor Angle:", left_motor.angle())
print("Right Motor Angle:", right_motor.angle())

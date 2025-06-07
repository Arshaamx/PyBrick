# 📥 Import necessary modules
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# ⚙️ Setup hub, motors, and color sensor
hub = PrimeHub()
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)  # Adjust if reversed
right_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.F)

# 📏 Calibrated values for line and background (adjust based on testing!)
BLACK = 9     # Reflected value on black line
WHITE = 47     # Reflected value on white background
THRESHOLD = (BLACK + WHITE) / 2  # Midpoint for edge following

# 🧠 Proportional gain (Kp) — tweak for performance
KP = 50

# 🚗 Base speed in degrees/second
BASE_SPEED = 200

# 🔁 Proportional Line Following Loop
def follow_line(duration_ms=2500):
    time_elapsed = 0
    while time_elapsed < duration_ms:
        # Measure reflected light
        light_value = color_sensor.reflection()

        # Calculate error and correction
        error = light_value - THRESHOLD
        correction = KP * error

        # Adjust motor speeds using P controller
        left_motor.run(BASE_SPEED - correction)
        right_motor.run(BASE_SPEED + correction)

        # Short delay to maintain timing
        wait(10)
        time_elapsed += 10

    # 🛑 Stop motors at the end
    left_motor.stop()
    right_motor.stop()

# ▶️ Run the line following function
follow_line()

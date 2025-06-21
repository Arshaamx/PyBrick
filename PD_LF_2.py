# 📥 Import necessary modules
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait

# ⚙️ Setup the robot hardware
hub = PrimeHub()
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)  # Left wheel
right_motor = Motor(Port.D)                             # Right wheel
color_sensor = ColorSensor(Port.F)                      # Color sensor pointing down

# 🎯 Calibrate the sensor
BLACK = 9     # Reflection on black line
WHITE = 47    # Reflection on white surface
THRESHOLD = (BLACK + WHITE) / 2  # Midpoint between black and white
 
 
# 🎚️ Control values (low = gentle, high = aggressive)
LOW_KP = 1.0
HIGH_KP = 3.0

LOW_KD = 0.3
HIGH_KD = 1.2

SLOW_SPEED = 150
FAST_SPEED = 400

# 🧮 Estimate the largest error we expect (for scaling)
MAX_ERROR = 30  # adjust if needed based on real testing

# 📘 This function does smart PD line following with gradual changes
def smart_pd_line_follow(duration_ms=5000):
    print("Starting line following...")
    last_error = 0
    time_elapsed = 0

    while time_elapsed < duration_ms:
        # 1️⃣ Get the brightness value from the color sensor
        reflection = color_sensor.reflection()

        # 2️⃣ Compare it to the threshold to see how far off we are
        error = reflection - THRESHOLD

        # 3️⃣ Derivative: how much the error is changing
        derivative = error - last_error
        last_error = error

        # 4️⃣ Normalize the error (range 0 to 1)
        norm_error = min(abs(error) / MAX_ERROR, 1)

        # 5️⃣ Gradually calculate KP, KD, and speed
        kp = LOW_KP + (HIGH_KP - LOW_KP) * norm_error
        kd = LOW_KD + (HIGH_KD - LOW_KD) * norm_error
        speed = FAST_SPEED - (FAST_SPEED - SLOW_SPEED) * norm_error

        # 6️⃣ PD formula to calculate correction
        correction = kp * error + kd * derivative

        # 7️⃣ Apply correction to motors
        left_motor.run(speed - correction)
        right_motor.run(speed + correction)

        # 8️⃣ Wait a bit before next check
        wait(10)
        time_elapsed += 10

    # 9️⃣ Stop the motors after done
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    print("Finished line following.")

# ▶️ Run the function!
smart_pd_line_follow(duration_ms=5000)

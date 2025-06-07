# -------------------------------------------------
# 🎯 GOAL: Use Gyro with PD and Encoder Control
# -------------------------------------------------
# ✅ Turns using PD control with gyro
# ✅ Moves straight using encoders with gyro correction and accel/decel
# ✅ Speeds and correction are now tuned for accurate and smooth motion

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait

# -------------------------------------------------
# ⚙️ Setup: Hub and Motor Configuration
# -------------------------------------------------

hub = PrimeHub()
left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)

# -------------------------------------------------
# 🔁 Gyro Turn with PD (Left or Right)
# -------------------------------------------------

def gyro_turn(target_angle, direction="right", turn_speed=100, kp=2.5, kd=1.2):
    """
    PD-controlled gyro turn.
    - target_angle: degrees to turn (e.g. 90)
    - direction: "right" or "left"
    - turn_speed: max turning speed
    """
    hub.imu.reset_heading(0)
    wait(100)  # Let gyro stabilize

    if direction == "left":
        target_angle = -target_angle

    last_error = 0

    while True:
        current_angle = hub.imu.heading()
        error = target_angle - current_angle
        derivative = error - last_error
        last_error = error

        power = kp * error + kd * derivative
        power = max(-turn_speed, min(turn_speed, power))  # Clamp speed

        left_motor.run(power)
        right_motor.run(-power)

        # DEBUG: print("Turn Angle:", current_angle, "Power:", power)

        if abs(error) < 1:
            break

        wait(10)

    left_motor.stop()
    right_motor.stop()

# -------------------------------------------------
# 🚗 Move Straight with PD Gyro + Accel/Decel
# -------------------------------------------------

def move_straight_with_gyro(degrees=720, max_speed=1000, kp=3, kd=1.8, accel_decel_ratio=0.15):
    """
    PD-stabilized straight driving using encoders and gyro.
    - degrees: distance to travel
    - max_speed: top speed
    - kp/kd: gyro correction constants
    - accel_decel_ratio: portion of motion for speed ramping
    """
    hub.imu.reset_heading(0)
    wait(100)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    last_error = 0
    min_speed = 60

    accel_end = degrees * accel_decel_ratio
    decel_start = degrees * (1 - accel_decel_ratio)

    while True:
        avg_deg = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2

        if avg_deg >= degrees:
            break

        # Acceleration / Deceleration logic
        if avg_deg < accel_end:
            current_speed = max_speed * (avg_deg / accel_end)
        elif avg_deg > decel_start:
            current_speed = max_speed * ((degrees - avg_deg) / (degrees - decel_start))
        else:
            current_speed = max_speed

        current_speed = max(current_speed, min_speed)

        # Gyro PD correction
        angle = hub.imu.heading()
        error = angle
        derivative = error - last_error
        last_error = error

        correction = kp * error + kd * derivative

        left_motor.run(current_speed - correction)
        right_motor.run(current_speed + correction)

        # DEBUG: print("Angle:", angle, "Corr:", correction, "Speed:", current_speed)

        wait(10)

    left_motor.stop()
    right_motor.stop()

# -------------------------------------------------
# 🧪 TEST EXAMPLES (REALISTIC SPEEDS)
# -------------------------------------------------

# Forward 3000 degrees (~5 wheel turns) at medium speed
move_straight_with_gyro(degrees=3000, max_speed=500)

# 90° left turn at safe speed
gyro_turn(90, direction="left", turn_speed=100)

# Forward 1100 degrees
move_straight_with_gyro(degrees=1100, max_speed=250)

# Another 90° left
gyro_turn(90, direction="left", turn_speed=100)

# Final straight segment
move_straight_with_gyro(degrees=3000, max_speed=300)

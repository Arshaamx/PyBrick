from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait

hub = PrimeHub()
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.F)

hub.imu.reset_heading(0)

def drive_straight_pid_with_scan(target_degrees, target_speed, scan_distance):
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    hub.imu.reset_heading(0)

    KP = 2.0
    KI = 0.0
    KD = 0.8

    integral = 0
    last_error = 0

    last_spot = -1

    while True:
        left_angle = left_motor.angle()
        right_angle = right_motor.angle()
        avg_angle = (abs(left_angle) + abs(right_motor.angle())) / 2

        if avg_angle >= target_degrees:
            break

        heading = hub.imu.heading()
        error = -heading
        integral += error
        derivative = error - last_error
        last_error = error

        correction = KP * error + KI * integral + KD * derivative

        left_motor.run(target_speed - correction)
        right_motor.run(target_speed + correction)

        # Scan every scan_distance
        current_spot = int(avg_angle // scan_distance)
        if current_spot != last_spot:
            detected = color_sensor.color()
            print(f"Spot {current_spot + 1} Color:", detected)
            last_spot = current_spot

        wait(10)

    # Finish with explicit run_target() to avoid angle error
    left_motor.run_target(target_speed, target_degrees, wait=True)
    right_motor.run_target(target_speed, target_degrees, wait=True)

def run_mission():
    drive_straight_pid_with_scan(
        target_degrees=1000,
        target_speed=200,
        scan_distance=200
    )
    print("Done.")

run_mission()

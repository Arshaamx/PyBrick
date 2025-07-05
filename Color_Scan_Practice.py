from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait

hub = PrimeHub()

# Motors
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D)

# Color sensor
color_sensor = ColorSensor(Port.F)

# Simple drive
def drive_straight_simple(target_degrees, target_speed):
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    while True:
        avg_angle = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2

        if avg_angle >= target_degrees:
            break

        left_motor.run(target_speed)
        right_motor.run(target_speed)

        wait(10)

    left_motor.stop()
    right_motor.stop()

# Scan using .color() instead of .rgb()
def scan_color():
    detected = color_sensor.color()
    print("Detected:", detected)

    if detected == Color.RED:
        return "RED"
    elif detected == Color.GREEN:
        return "GREEN"
    else:
        return "UNKNOWN"

# Mission
def run_mission():
    spots = []

    for i in range(5):
        print(f"Spot {i+1}")

        drive_straight_simple(target_degrees=200, target_speed=200)

        color = scan_color()
        spots.append(color)
        print(f"Spot {i+1}:", color)

        wait(500)

    hub.display.clear()
    hub.display.text("Results:")
    for idx, color in enumerate(spots):
        if color != "UNKNOWN":
            hub.display.text(f"{idx+1}:{color}")

    print("Done:", spots)

run_mission()

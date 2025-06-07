from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Button, Port
from pybricks.tools import wait

# Setup hub and color sensor
hub = PrimeHub()
sensor = ColorSensor(Port.F)

# Initialize variables
black_value = None
white_value = None

# Print instructions
print("📌 Place sensor on BLACK line and press RIGHT button.")
print("📌 Place sensor on WHITE background and press LEFT button.")
print("➡️ Press CENTER button to exit.")

# Main loop
while True:
    # Read current sensor value
    current_reflection = sensor.reflection()

    # Check if button is pressed
    buttons = hub.buttons.pressed()

    if Button.RIGHT in buttons:
        black_value = current_reflection
        print("🖤 BLACK value recorded:", black_value)
        wait(500)

    elif Button.LEFT in buttons:
        white_value = current_reflection
        print("⚪ WHITE value recorded:", white_value)
        wait(500)

    elif Button.CENTER in buttons:
        print("✅ Exiting program.")
        if black_value is not None and white_value is not None:
            threshold = (black_value + white_value) / 2
            print("📊 Suggested Threshold:", threshold)
        break

    wait(100)

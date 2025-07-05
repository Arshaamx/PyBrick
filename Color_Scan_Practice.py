# Import necessary modules for SPIKE Prime
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait

# Initialize the hub (the SPIKE Prime brain)
hub = PrimeHub()

# Initialize motors:
# Left motor is on Port C and spins COUNTERCLOCKWISE to match robot build
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

# Right motor is on Port D and spins normally
right_motor = Motor(Port.D)

# Initialize color sensor on Port F
color_sensor = ColorSensor(Port.F)

# Reset the hub's built-in gyro (IMU) so heading starts at 0 degrees
hub.imu.reset_heading()


# Function: Drive straight for a specific distance (degrees) with gyro correction (PID)
def drive_straight_pid(target_degrees, target_speed):
    """
    Drive forward for 'target_degrees' using a gyro-based PID controller.
    - target_degrees: how far to drive (wheel rotation in degrees)
    - target_speed: how fast to drive (degrees per second)
    """
    # Reset motor angles to zero
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    # Also reset gyro heading to zero
    hub.imu.reset_heading()

    # Set up PID controller constants for heading correction
    KP = 2.0  # Proportional gain: reacts to how far off the robot is from straight
    KI = 0.0  # Integral gain: corrects small drift over time (optional here)
    KD = 0.8  # Derivative gain: dampens sudden swings and helps smooth turns

    # Start with no integral or derivative error yet
    integral = 0
    last_error = 0

    # Keep looping until we reach the target distance
    while True:
        # Get current rotation for each motor
        left_angle = left_motor.angle()
        right_angle = right_motor.angle()

        # Calculate the average rotation for both wheels
        avg_angle = (abs(left_angle) + abs(right_angle)) / 2

        # Check if we've reached our target distance
        if avg_angle >= target_degrees:
            break  # Exit the loop when done

        # Get current heading (angle) from the gyro
        heading = hub.imu.heading()

        # Calculate how far off we are from perfectly straight (error)
        error = -heading  # Negative because we want to correct in the opposite direction

        # Calculate integral (adds up small errors over time)
        integral += error

        # Calculate derivative (how fast the error is changing)
        derivative = error - last_error
        last_error = error  # Save current error for next loop

        # Combine P, I, D parts into one correction value
        correction = KP * error + KI * integral + KD * derivative

        # Apply correction to motor speeds:
        # Turn left motor slower and right motor faster (or vice versa)
        left_motor.run(target_speed - correction)
        right_motor.run(target_speed + correction)

        # Small pause before the next loop
        wait(10)

    # Stop both motors when the loop ends
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)


# Function: Scan for color using RGB mode
def scan_color():
    """
    Uses the color sensor in RGB mode to check the surface color.
    Returns 'RED', 'GREEN', or 'UNKNOWN' based on simple thresholds.
    """
    # Read raw RGB values from the sensor
    rgb = color_sensor.rgb()
    r, g, b = rgb

    print("RGB:", rgb)  # Print for debugging

    # Simple logic: if red is strongest, it's RED; if green is strongest, it's GREEN
    if r > g and r > b and r > 30:
        return "RED"
    elif g > r and g > b and g > 30:
        return "GREEN"
    else:
        return "UNKNOWN"


# Main task: visit 5 spots, scan for color at each, and show results
def run_mission():
    """
    Example mission:
    1) Drive forward to each spot.
    2) Scan the color.
    3) Remember which spots have color.
    4) Print results to console.
    5) Show found spots on the hub's display.
    """

    # List to store each spot's result
    spots = []

    # Visit 5 spots
    for i in range(5):
        print(f"Spot {i+1}")

        # Drive forward for 200 degrees at 200 deg/sec (adjust as needed)
        drive_straight_pid(target_degrees=200, target_speed=200)

        # Scan the color at this spot
        color = scan_color()

        # Save color to list
        spots.append(color)

        # Print result for this spot
        print(f"Spot {i+1}: {color}")

        # Wait a moment before next spot
        wait(500)

    # Clear hub screen and show results
    hub.display.clear()
    hub.display.text("Summary:")

    for idx, color in enumerate(spots):
        if color != "UNKNOWN":
            # Show spot number and color if it found something
            hub.display.text(f"{idx+1}: {color}")

    print("All spots checked:", spots)


# Run the mission!
run_mission()

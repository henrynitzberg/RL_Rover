# Import the necessary libraries
import RPi.GPIO as GPIO
import time
import signal
import sys

# --- Configuration ---
# Define the GPIO pin number to activate.
# Use the BCM pin numbering scheme (refer to your Raspberry Pi model's pinout).
# Example: GPIO 4 corresponds to physical pin 11 on most models.
GPIO_PINS = [17, 26]

# --- Script Logic ---

# Function to handle script termination gracefully (e.g., Ctrl+C)
def signal_handler(sig, frame):
    """Handles termination signals and ensures GPIO cleanup."""
    print('\nTermination signal received. Cleaning up GPIO...')
    GPIO.cleanup()
    print('GPIO cleanup complete. Exiting.')
    sys.exit(0)

# Register the signal handler for SIGINT (Ctrl+C) and SIGTERM
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

try:
    # Set the GPIO mode to BCM
    # BCM refers to the Broadcom SOC channel numbers.
    # Alternatively, you can use GPIO.BOARD for physical pin numbers.
    GPIO.setmode(GPIO.BCM)
    print(f"Using BCM GPIO numbering scheme.")

    # Disable warnings about channels already in use (optional)
    GPIO.setwarnings(False)
    print("GPIO warnings disabled.")

    # Set up the specified GPIO pin as an output pin
    for GPIO_PIN in GPIO_PINS:
        GPIO.setup(GPIO_PIN, GPIO.OUT)
        print(f"Setting up GPIO pin {GPIO_PIN} as an output.")

        # Set the GPIO pin to HIGH (activate it)
        GPIO.output(GPIO_PIN, GPIO.HIGH)
        print(f"GPIO pin {GPIO_PIN} activated (set to HIGH).")
        print("Pin will remain active. Press Ctrl+C to stop the script and clean up.")

    # Keep the script running indefinitely
    # The pin state is maintained as long as the script runs.
    # time.sleep(1) is used to prevent the loop from consuming 100% CPU.
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    # This block is technically redundant because of the signal handler,
    # but it's good practice to include it.
    print("\nKeyboardInterrupt detected (Ctrl+C). Cleaning up...")
    ALL_GPIO_PINS = [
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27]

    for pin in ALL_GPIO_PINS:
        try:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        except Exception as e:
            print(f"Could not deactivate GPIO pin {pin}: {e}")

    print("All GPIO pins set to input with no pull resistors.")
    print("GPIO cleanup complete.")

except Exception as e:
    # Catch any other potential errors during setup
    print(f"An error occurred: {e}")
    print("Attempting GPIO cleanup...")
    GPIO.cleanup()
    print("GPIO cleanup complete.")

finally:
    # This block ensures cleanup happens even if unexpected errors occur
    # that aren't caught by the specific except blocks above.
    # Note: In typical scenarios with the signal handler, this might not
    # always be reached if the exit happens within the handler.
    # However, it's a safeguard.
    # Ensure cleanup is called if the script exits for any reason other than the handled signals.
    # Check if GPIO has been set up before cleaning up
    if GPIO.getmode() is not None: # Check if GPIO mode was set
         print("Performing final GPIO cleanup...")
         GPIO.cleanup()
         print("Final GPIO cleanup complete.")

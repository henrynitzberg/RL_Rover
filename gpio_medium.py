# Import the necessary libraries
import RPi.GPIO as GPIO
import time
import signal
import sys

# --- Configuration ---
GPIO_PINS = [17, 26]      # The GPIO pin to use (BCM numbering)
PULSE_FREQUENCY = 40 # Frequency in Hertz (Hz) - 20 times per second
DUTY_CYCLE = 50     # Percentage (0.0 to 100.0) - 50% for equal HIGH/LOW time

# --- Global variable for PWM object ---
pwm = None

# --- Script Logic ---

# Function to handle script termination gracefully
def signal_handler(sig, frame):
    """Handles termination signals, stops PWM, and ensures GPIO cleanup."""
    print('\nTermination signal received. Cleaning up...')
    if pwm:
        print("Stopping PWM...")
        pwm.stop()
    GPIO.cleanup()
    print('GPIO cleanup complete. Exiting.')
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

try:
    # Set the GPIO mode
    GPIO.setmode(GPIO.BCM)
    print(f"Using BCM GPIO numbering scheme.")

    # Disable warnings
    GPIO.setwarnings(False)
    print("GPIO warnings disabled.")

    for GPIO_PIN in GPIO_PINS:
        # Set up the pin as an output
        GPIO.setup(GPIO_PIN, GPIO.OUT)
        print(f"Setting up GPIO pin {GPIO_PIN} as an output.")

        # Create PWM instance
        # GPIO.PWM(channel, frequency)
        # We set the frequency directly to 20 Hz
        pwm = GPIO.PWM(GPIO_PIN, PULSE_FREQUENCY)
        print(f"Created PWM instance on pin {GPIO_PIN} with frequency {PULSE_FREQUENCY} Hz.")

        # Start PWM
        # pwm.start(dutycycle) - dutycycle is % (0.0 to 100.0)
        # A 50% duty cycle means the pin is HIGH for half the period and LOW for the other half.
        pwm.start(DUTY_CYCLE)
        print(f"PWM started with duty cycle {DUTY_CYCLE}%. Pin {GPIO_PIN} is now pulsing at {PULSE_FREQUENCY} Hz.")
        print("Press Ctrl+C to stop the script and clean up.")

    # Keep the script running while PWM operates in the background
    while True:
        # The time.sleep(1) here doesn't affect the PWM frequency.
        # It just prevents this main loop from consuming 100% CPU.
        # The RPi.GPIO library handles the PWM pulsing in a separate thread.
        time.sleep(1)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected (Ctrl+C). Cleaning up...")
    # Cleanup handled by finally block or signal handler

except Exception as e:
    print(f"An error occurred: {e}")
    print("Attempting cleanup...")
    # Cleanup handled by finally block

finally:
    # This block ensures cleanup happens reliably
    print("Performing final cleanup...")
    if pwm:
        print("Stopping PWM in finally block...")
        pwm.stop() # Stop PWM if it was started
    if GPIO.getmode() is not None: # Check if GPIO mode was set
        print("Cleaning up GPIO in finally block...")
        GPIO.cleanup()
        print("Final GPIO cleanup complete.")
    else:
        print("GPIO mode not set, no cleanup needed.")
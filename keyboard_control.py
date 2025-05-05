import RPi.GPIO as GPIO
import sys
import termios
import tty
import time
import signal
import select

# --- Configuration ---
LEFT_PIN = 26
RIGHT_PIN = 17

# --- Setup ---

def signal_handler(sig, frame):
    print('\nTermination signal received. Cleaning up GPIO...')
    GPIO.cleanup()
    print('GPIO cleanup complete. Exiting.')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in [LEFT_PIN, RIGHT_PIN]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)  # Start with pins off

def deactivate_all():
    GPIO.output(LEFT_PIN, GPIO.LOW)
    GPIO.output(RIGHT_PIN, GPIO.LOW)

def get_keypress_nonblocking():
    """Non-blocking read of one character, returns None if no key pressed."""
    dr, dw, de = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def main_loop():
    print("Ready. Hold W (both motors), A (left only), D (right only). Press Q to quit.")

    # Save terminal settings
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # Set terminal to cbreak mode (no Enter needed)

    try:
        active_key = None
        while True:
            key = get_keypress_nonblocking()

            if key:
                key = key.lower()
                if key == 'q':
                    print("Quitting...")
                    break
                elif key in ['w', 'a', 'd']:
                    active_key = key

            if active_key == 'w':
                GPIO.output(LEFT_PIN, GPIO.HIGH)
                GPIO.output(RIGHT_PIN, GPIO.HIGH)
            elif active_key == 'a':
                GPIO.output(LEFT_PIN, GPIO.HIGH)
                GPIO.output(RIGHT_PIN, GPIO.LOW)
            elif active_key == 'd':
                GPIO.output(LEFT_PIN, GPIO.LOW)
                GPIO.output(RIGHT_PIN, GPIO.HIGH)
            else:
                deactivate_all()

            time.sleep(0.05)  # Check every 50ms

            # Stop motors if no key is being pressed
            if not sys.stdin.isatty():  # Edge case: stdin not a terminal
                deactivate_all()

            # No keypress detected, assume key released
            if key is None:
                active_key = None

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        deactivate_all()

if __name__ == "__main__":
    try:
        setup_gpio()
        main_loop()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        GPIO.cleanup()

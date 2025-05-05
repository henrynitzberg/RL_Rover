import RPi.GPIO as GPIO
import sys
import termios
import tty
import time
import signal
import select

# --- Configuration ---
RIGHT_PIN       = 17
LEFT_PIN        = 26
PULSE_FREQUENCY = 40    # Hz
STUTTER_DC      = 50    # duty cycle % for stutter

# --- Cleanup on exit ---
def signal_handler(sig, frame):
    print("\nExiting, cleaning up GPIO...")
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# --- GPIO & PWM setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RIGHT_PIN, GPIO.OUT)
GPIO.setup(LEFT_PIN, GPIO.OUT)

pwm_right = GPIO.PWM(RIGHT_PIN, PULSE_FREQUENCY)
pwm_left  = GPIO.PWM(LEFT_PIN,  PULSE_FREQUENCY)

# start both at 0% (off)
pwm_right.start(0)
pwm_left.start(0)

# --- Non?blocking key read setup ---
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

def get_key_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1).lower()
    return None

print("Hold a/d/w to run motors, q to quit.")

try:
    while True:
        key = get_key_nonblocking()

        if key == 'q':
            break

        # Default: both off
        dc_right = 0
        dc_left  = 0

        if key == 'w':
            # both stutter
            dc_right = STUTTER_DC
            dc_left  = STUTTER_DC

        elif key == 'd':
            # right full-on, left stutter
            dc_right = 100
            dc_left  = STUTTER_DC

        elif key == 'a':
            # left full-on, right stutter
            dc_left  = 100
            dc_right = STUTTER_DC

        # apply duty-cycles
        pwm_right.ChangeDutyCycle(dc_right)
        pwm_left.ChangeDutyCycle(dc_left)

        time.sleep(0.02)    # 20 ms loop ? responsive without eating CPU

finally:
    # restore terminal & cleanup
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()
    print("Cleaned up, bye!")

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
STUTTER_DC_LEFT = 50    # duty cycle % for stutter
STUTTER_DC_RIGHT = 75
MOVE_TIME       = 1.0   # seconds to move forward
TURN_TIME       = 1.0   # seconds to turn left/right

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

# --- Non-blocking key read setup ---
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

def get_key_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1).lower()
    return None

# Function to run motors for a specific duration
def run_motors(dc_right, dc_left, duration):
    pwm_right.ChangeDutyCycle(dc_right)
    pwm_left.ChangeDutyCycle(dc_left)
    time.sleep(duration)
    pwm_right.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)

print("Press arrow keys, a, d, or w to control motors, q to quit.")
print("↑: forward for 1 second")
print("a: left tread only for 0.5 seconds")
print("d: right tread only for 0.5 seconds")
print("w: both treads stutter")

try:
    while True:
        key = get_key_nonblocking()
        
        if key == 'q':
            break
            
        elif key == '\x1b':  # Escape sequence for arrow keys
            # Read the next two characters
            seq1 = sys.stdin.read(1)
            seq2 = sys.stdin.read(1)
            
            if seq1 == '[':  # Arrow keys send ESC[A, ESC[B, etc.
                if seq2 == 'A':  # Up arrow
                    print("Moving forward for 1 second")
                    run_motors(STUTTER_DC, STUTTER_DC, MOVE_TIME)
        
        elif key == 'w':
            # Manual control - both stutter for move time
            run_motors(STUTTER_DC_RIGHT, STUTTER_DC_LEFT, MOVE_TIME)
            # Reset when key is released (next loop iteration)
            
        elif key == 'd':
            # Right turn with stuttering for half a second
            #print("Right turn for 0.5 seconds")
            run_motors((2*STUTTER_DC_LEFT + 3*STUTTER_DC_RIGHT)/5, 0, TURN_TIME)
            
        elif key == 'a':
            # Left turn with stuttering for half a second
            #print("Left turn for 0.5 seconds")
            run_motors(0, STUTTER_DC_LEFT, TURN_TIME)
            
        else:
            # No key or unrecognized key - motors off
            pwm_right.ChangeDutyCycle(0)
            pwm_left.ChangeDutyCycle(0)
            
        time.sleep(0.02)  # 20 ms loop - responsive without eating CPU

finally:
    # restore terminal & cleanup
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()
    print("Cleaned up, bye!")
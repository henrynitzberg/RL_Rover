import numpy as np
import stable_baselines3 as sb3
from env import simEnv
from Lidar_Reader import get_lidar_scans
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

def run_motors(dc_right, dc_left, duration):
    pwm_right.ChangeDutyCycle(dc_right)
    pwm_left.ChangeDutyCycle(dc_left)
    time.sleep(duration)
    pwm_right.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)

def execute_action(action):
    #action 1 is turn left
    if action == 1:
        run_motors(0, STUTTER_DC_LEFT, TURN_TIME)
    #action 2 is turn right
    elif action == 2:
        run_motors((2*STUTTER_DC_LEFT + 3*STUTTER_DC_RIGHT)/5, 0, TURN_TIME)
    #0 is go forward
    elif action == 0:
        run_motors(STUTTER_DC_RIGHT, STUTTER_DC_LEFT, MOVE_TIME)
    else:
        print("Invalid action")


map_height = 7
map_width = 7
model = sb3.PPO.load(f"models/ppo_sim-{map_height}-{map_width}")

env = simEnv(map_height=map_height, map_width=map_width, render_time=.1)

#get lidar data as a list of angle, distance pairs (get sufficient number of data points, average each subscan into one vector)
lidar_data = get_lidar_scans(num_scans=1)[0]

done = False
robot_angle = 30
while True:
    angles_buffer = []
    dists_buffer = []
    while len(angles_buffer) < 24:
        
        angles = lidar_data["angles"]
        distances = lidar_data["distances"]

        angles_buffer.append(np.average(angles))
        dists_buffer.append(np.average(distances))

    obs = {
            "agent_angle": np.array([np.deg2rad(robot_angle)]),
            "angles": angles_buffer,
            "dists": dists_buffer,
        }

    action, _ = model.predict(obs, deterministic=True)
    execute_action(action)
    if action == 1:
        robot_angle += 30
        if robot_angle > 360:
            robot_angle -= 360
    if action == 2:
        robot_angle -= 30
        if robot_angle < 0:
            robot_angle += 360

    
    
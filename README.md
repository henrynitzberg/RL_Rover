# Installing requirements:
The requirements are outlined in the 'requirements.txt' file.
Recommended installation instructions (using venv on Linux):
`cd path/to/RL_Rover`
`mkdir env && python -m venv env && source env/bin/activate && pip install -r requirements.txt`

To visualize model execution in simulation:
`python simulator/test.py`

To train a model:
`python simulator/train.py`

We encourage fiddling with the 'map_height' and 'map_width' variables to see how they effect performance!

Notes:
* Lidar_Reader.py/.cpp files are only useful for the LD20 LIDAR sensor.
* keyboard_control.py, medium_control.py, timed_control.py, stop_gpio.py and simulator/drive_with_policy.py are only to be used on the physical robot.
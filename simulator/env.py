import numpy as np
import matplotlib.pyplot as plt
import cv2

import gymnasium as gym
from gymnasium import spaces

import stable_baselines3 as sb3


# make map
MAP_DIMS = (10, 15)


def make_map():
    map = np.zeros(MAP_DIMS, dtype=int)

    map[0, : MAP_DIMS[1]] = 1
    map[MAP_DIMS[0] - 1, : MAP_DIMS[1]] = 1
    map[: MAP_DIMS[0], 0] = 1
    map[: MAP_DIMS[0], MAP_DIMS[1] - 1] = 1

    return map


def state_to_image(map, state, goal_loc=None):
    # Convert the map to a binary image
    img = np.zeros((map.shape[0], map.shape[1]), dtype=np.uint8)
    img[map == 1] = 0  # Set walls to white
    img[map == 0] = 255  # Set free space to black

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    if "loc" in state:
        agent_loc = state["loc"]
        # print(f"placing agent at: {agent_loc}")
        img[agent_loc[0], agent_loc[1]] = (0, 255, 0)
    if goal_loc is not None:
        img[goal_loc[0], goal_loc[1]] = (255, 0, 0)
    if "angles" in state and "dists" in state:
        angles = state["angles"]
        dists = state["dists"]
        for angle, dist in zip(angles, dists):
            x = int(agent_loc[0] + dist * np.cos(angle))
            y = int(agent_loc[1] + dist * np.sin(angle))
            img[x, y] = (0, 0, 255)

    goal_width = 400
    scale_factor = int(goal_width / MAP_DIMS[0])
    new_dims = (MAP_DIMS[1] * scale_factor, MAP_DIMS[0] * scale_factor)

    img = cv2.resize(img, new_dims, interpolation=cv2.INTER_NEAREST)
    if "loc" in state and "angles" in state and "dists" in state:
        agent_loc = state["loc"]
        angles = state["angles"]
        dists = state["dists"]

        scaled_loc = (
            int((agent_loc[0] + 0.5) * scale_factor),
            int((agent_loc[1] + 0.5) * scale_factor),
        )

        for angle, dist in zip(angles, dists):
            x = int(
                (agent_loc[0] * scale_factor) + ((dist * scale_factor) * np.cos(angle))
            )
            y = int(
                (agent_loc[1] * scale_factor) + ((dist * scale_factor) * np.sin(angle))
            )
            cv2.line(
                img,
                (scaled_loc[1], scaled_loc[0]),
                (y, x),
                (0, 0, 255),
                1,
            )

    return img


def gen_lidar_data(map, loc, spread=(5, 7)):
    start_angle = np.random.uniform(0, 2 * np.pi)
    spread = np.random.uniform(np.deg2rad(spread[0]), np.deg2rad(spread[1]))
    num_rays = 12
    angles = np.linspace(start_angle, start_angle + spread, num_rays)

    dists = np.zeros(num_rays, dtype=int)
    # Use Bresenham's Line Algorithm
    for i, angle in enumerate(angles):
        x, y = loc
        dx = np.cos(angle)
        dy = np.sin(angle)

        distance = 0
        while True:
            x += dx
            y += dy
            distance += 1

            grid_x = int(x)
            grid_y = int(y)

            if (
                grid_x < 0
                or grid_x >= map.shape[0]
                or grid_y < 0
                or grid_y >= map.shape[1]
            ):
                break
            if map[grid_x, grid_y] == 1:
                break

        dists[i] = distance

    return angles, dists


# show map
map = make_map()

# loc = (5, 5)
# goal_loc = (8, 8)
# map[goal_loc[0], goal_loc[1]] = 1
# angles = []
# dists = []
# for i in range(10):
#     new_angles, new_dists = gen_lidar_data(map, loc)
#     angles = angles + new_angles.tolist()
#     dists = dists + new_dists.tolist()

# state = {}
# state["loc"] = loc
# state["angles"] = angles
# state["dists"] = dists
# img = state_to_image(map, state, goal_loc=goal_loc)
# cv2.imshow("map", img)
# cv2.waitKey(0)


class simEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    def __init__(self, map):
        super().__init__()

        self.map = map
        self.drivable_terrain = np.argwhere(map == 0)
        print(f"drivable terrain: {self.drivable_terrain}")

        self.action_space = spaces.Discrete(
            3
        )  # 3 possible actions: forward, left, right

        self.turn_angle = np.deg2rad(30)
        self.agent_speed = 1  # configure but keep constant
        self.dt = 1  # seconds between actions

        self.num_scans = int(self.dt * 6)
        self.observation_space = spaces.Dict(
            {
                "agent_angle": spaces.Box(
                    low=0,
                    high=2 * np.pi,
                    shape=(1,),
                    dtype=np.float32,
                ),
                "angles": spaces.Box(
                    low=0,
                    high=2 * np.pi,
                    shape=(self.num_scans * 12,),
                    dtype=np.float32,
                ),
                "dists": spaces.Box(
                    low=0,
                    high=(max(MAP_DIMS[0], MAP_DIMS[1])),
                    shape=(self.num_scans * 12,),
                    dtype=np.float32,
                ),
            }
        )

        self.goal_loc = (-1, -1)
        self.agent_loc = (-1, -1)
        self.agent_angle = -1
        self.obs = None

    # 0 is forward, 1 is left, 2 is right
    def step(self, action):
        reward = 0
        if action == 0:
            # move forward
            dx = self.agent_speed * np.cos(self.agent_angle)
            dy = self.agent_speed * np.sin(self.agent_angle)
            self.agent_loc[0] += dx
            self.agent_loc[1] += dy
            loc_estimate = (int(self.agent_loc[0]), int(self.agent_loc[1]))
            # check if goal
            if loc_estimate[0] == self.goal_loc[0] and loc_estimate[1] == self.goal_loc[1]:
                print("GOAL REACHED")
                reward = 100
                terminated = True
            # check if out of bounds
            elif (
                loc_estimate[0] < 0
                or loc_estimate[0] >= self.map.shape[0]
                or loc_estimate[1] < 0
                or loc_estimate[1] >= self.map.shape[1]
            ):
                reward = -100
                terminated = True
            elif self.map[loc_estimate[0], loc_estimate[1]] == 1:
                reward = -100
                terminated = True
            else:
                reward = -1
                terminated = False
        elif action == 1:
            # turn left
            self.agent_angle += self.turn_angle
            if self.agent_angle > 2 * np.pi:
                self.agent_angle -= 2 * np.pi
            reward = -1
            terminated = False
        elif action == 2:
            # turn right
            self.agent_angle -= self.turn_angle
            if self.agent_angle < 0:
                self.agent_angle += 2 * np.pi
            reward = -1
            terminated = False
        loc_estimate = (int(self.agent_loc[0]), int(self.agent_loc[1]))
        # check if goal
        if loc_estimate[0] == self.goal_loc[0] and loc_estimate[1] == self.goal_loc[1]:
            print("GOAL REACHED")
            reward = 100
            terminated = True
        # check if out of bounds
        elif (
            loc_estimate[0] < 0
            or loc_estimate[0] >= self.map.shape[0]
            or loc_estimate[1] < 0
            or loc_estimate[1] >= self.map.shape[1]
        ):
            reward = -100
            terminated = True
        elif self.map[loc_estimate[0], loc_estimate[1]] == 1:
            reward = -100
            terminated = True
        
        print(f"agent loc: {self.agent_loc}, angle: {np.rad2deg(self.agent_angle)}")
        print(f"action: {action}, reward: {reward}")

        self._get_obs()
        return self.obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        # choose random start and goal locations
        loc_is = np.random.randint(0, len(self.drivable_terrain), 2)
        self.agent_loc = [float(self.drivable_terrain[loc_is[0]][0]), float(self.drivable_terrain[loc_is[0]][1])]
        # self.agent_loc = [0,0]
        self.goal_loc = (self.drivable_terrain[loc_is[1]][0], self.drivable_terrain[loc_is[1]][1])
        self.agent_angle = np.random.uniform(0, 2 * np.pi)
        print("RESETTING ENVIRONMENT")
        print(
            f"agent loc: {self.agent_loc}, goal loc: {self.goal_loc}, angle: {np.rad2deg(self.agent_angle)}"
        )

        self.curr_map = np.copy(self.map)
        self.curr_map[self.goal_loc[0], self.goal_loc[1]] = 1

        self._get_obs()
        return self.obs, {}

    def render(self):
        # Render the environment to the screen
        state = {}
        state["loc"] = (int(self.agent_loc[0]), int(self.agent_loc[1]))
        state["angles"] = self.obs["angles"]
        state["dists"] = self.obs["dists"]
        state["goal_loc"] = self.goal_loc
        img = state_to_image(self.map, state, self.goal_loc)
        cv2.imshow("map", img)
        cv2.waitKey(self.dt * 100)

    def close(self):
        pass

    def _get_obs(self):
        # lidar works at about 6hz
        angles = []
        dists = []
        for i in range(self.num_scans):
            new_angles, new_dists = gen_lidar_data(self.curr_map, self.agent_loc)
            angles = angles + new_angles.tolist()
            dists = dists + new_dists.tolist()
        angles = np.array(angles)
        dists = np.array(dists)

        obs = {
            "agent_angle": np.array([self.agent_angle]),
            "angles": angles,
            "dists": dists,
        }

        self.obs = obs


map = make_map()
env = simEnv(map)
model = sb3.PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=10000)
model.save("ppo_simEnv")
obs, info = env.reset()
done = False
max_steps = 100
for _ in range(max_steps):
    if done:
        break
    action, _states = model.predict(obs, deterministic=False)
    obs, reward, done, _, _ = env.step(action)
    # print(f"action: {action}, reward: {reward}")
    env.render()

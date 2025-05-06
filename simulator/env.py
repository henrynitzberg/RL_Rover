import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# walkable cells: 0
# walls: 1
# goal: 2
custom_map = ListedColormap(["white", "black", "green"])


def generate_gridworld(num_rows, num_cols, goal_loc=(1, 1)):
    grid = np.zeros((num_rows, num_cols)) + 1
    grid[1:-1, 1:-1] = 0  # walkable cells
    grid[goal_loc] = 2  # goal cell
    return grid


# grid is the gridworld
# state is a dictionary representing information about the agent's state
# wait is the time to wait before closing the plot
def plot_gridworld(grid, state={}, wait=-1):
    fig, ax = plt.subplots()
    ax.imshow(grid, cmap=custom_map)

    # Set major ticks at cell boundaries
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1))
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1))

    # Add gridlines
    ax.grid(color="black", linestyle="-", linewidth=1)

    # Remove tick labels
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    # Turn off axis ticks
    ax.tick_params(axis="both", which="both", length=0)

    if "agent_position" in state:
        (agent_row, agent_col) = state["agent_position"]
        # Draw red dot for agent
        ax.plot(agent_col, agent_row, "ro", markersize=20)
        if "agent_angle" in state:
            angle = state["agent_angle"]
            # Draw arrow for agent direction
            dx = np.cos(angle)
            dy = np.sin(angle)
            ax.arrow(
                agent_col,
                agent_row,
                dx,
                dy,
                head_width=0.3,
                head_length=0.5,
                fc="red",
                ec="red",
            )

    if "angles" in state and "dists" in state:
        angles = state["angles"]
        dists = state["dists"]
        for i, angle in enumerate(angles):
            # Draw lidar rays
            dx = dists[i] * np.cos(angle)
            dy = dists[i] * np.sin(angle)
            ax.plot(
                [agent_col, agent_col + dx],
                [agent_row, agent_row + dy],
                "b-",
                linewidth=2,
            )

    plt.draw()
    if wait > 0:
        plt.waitforbuttonpress(timeout=wait)
    else:
        plt.waitforbuttonpress()  # waits until a key is pressed
    plt.close()


def get_lidar_scans(grid, agent_loc, spread=(5, 7)):
    start_angle = np.random.uniform(0, 2 * np.pi)
    # start_angle = 0
    spread = np.random.uniform(np.deg2rad(spread[0]), np.deg2rad(spread[1]))
    num_rays = 12
    angles = np.linspace(start_angle, start_angle + spread, num_rays)

    dists = np.zeros(num_rays, dtype=int)
    # Use Bresenham's Line Algorithm
    for i, angle in enumerate(angles):
        row = agent_loc[0]
        col = agent_loc[1]

        dx = np.cos(angle)
        dy = np.sin(angle)

        distance = 0
        while True:
            row += dy
            col += dx
            distance += 1

            grid_row = int(row)
            grid_col = int(col)

            if (
                grid_row < 0
                or grid_row >= grid.shape[0]
                or grid_col < 0
                or grid_col >= grid.shape[1]
            ):
                break
            if grid[grid_row, grid_col] == 1 or grid[grid_row, grid_col] == 2:
                break

        dists[i] = distance

    return angles, dists


# world = generate_gridworld(10, 10, (5, 5))
# state = {}  # Example state with agent at (1, 1)
# state["agent_position"] = (1, 1)
# state["agent_angle"] = np.pi / 6 # 30 degrees
# angles = []
# dists = []
# for i in range(1000):
#     angles_, dists_ = get_lidar_scans(world, state["agent_position"], (5, 7))
#     angles += list(angles_)
#     dists += list(dists_)
# print(angles)
# state["angles"] = angles
# state["dists"] = dists
# plot_gridworld(world, state, wait=10)

import gymnasium as gym
from gymnasium import spaces

import stable_baselines3 as sb3


class simEnv(gym.Env):
    def __init__(self, map_height=5, map_width=5, render_time=.1):
        super().__init__()

        self.map_height = map_height
        self.map_width = map_width
        assert (
            self.map_height > 2 and self.map_width > 2
        ), "Map size must be greater than 2 in both dimensions"

        self.action_space = spaces.Discrete(
            3
        )  # 3 possible actions: forward, left, right

        self.turn_angle = np.deg2rad(30)
        self.agent_speed = 1  # configure but keep constant
        self.dt = 4  # seconds between actions
        self.render_time = render_time
        self.lidar_hz = 6
        self.num_beams = 12

        self.observation_space = spaces.Dict(
            {
                "agent_angle": spaces.Box(
                    low=0,
                    high=2 * np.pi,
                    shape=(1,),
                    dtype=np.float32,
                ),
                # "agent_loc": spaces.Box(
                #     low=0,
                #     high=max(self.map_height, self.map_width),
                #     shape=(2,),
                #     dtype=np.float32,
                # ),
                # "goal_loc": spaces.Box(
                #     low=0,
                #     high=max(self.map_height, self.map_width),
                #     shape=(2,),
                #     dtype=np.int32,
                # ),
                "angles": spaces.Box(
                    low=0,
                    high=2 * np.pi,
                    shape=(self.lidar_hz * self.dt,),
                    dtype=np.float32,
                ),
                "dists": spaces.Box(
                    low=0,
                    high=np.sqrt(
                        (self.map_height - 1) ** 2 + (self.map_width - 1) ** 2
                    ),
                    shape=(self.lidar_hz * self.dt,),
                    dtype=np.float32,
                ),
            }
        )

        # the following variables are set in reset()
        self.map = None
        self.goal_loc = None
        # these variables can also be changed in the step() function
        self.agent_loc = None  # represented as np.array([row, col])
        self.agent_angle = None  # represented as radians
        self.obs = None

    # 0 is forward, 1 is left, 2 is right
    def step(self, action):
        reward = -1
        terminated = False

        new_loc = self.agent_loc.copy()
        if action == 0:  # forward
            dx = self.agent_speed * np.cos(self.agent_angle)
            dy = self.agent_speed * np.sin(self.agent_angle)
            new_loc = self.agent_loc + np.array([dy, dx])
            self.agent_loc = new_loc
        elif action == 1:
            # turn left
            self.agent_angle -= self.turn_angle
            reward = -0.7
        elif action == 2:
            # turn right
            self.agent_angle += self.turn_angle
            reward = -0.7

        # wrap angle
        self.agent_angle = self.agent_angle % (2 * np.pi)

        estimated_loc = self.estimate_loc()

        # check if out of bounds
        if (
            estimated_loc[0] < 0
            or estimated_loc[0] >= self.map_height
            or estimated_loc[1] < 0
            or estimated_loc[1] >= self.map_width
        ):
            reward = -10
            terminated = True
        elif self.map[estimated_loc[0], estimated_loc[1]] == 1:
            # hit a wall
            reward = -10
            terminated = True
        elif self.map[estimated_loc[0], estimated_loc[1]] == 2:
            # reached the goal
            reward = 100
            terminated = True

        self.agent_loc = new_loc

        self._get_obs()
        return self.obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        # get random goal location and agent start location
        row_options = np.arange(1, self.map_height - 1)
        col_options = np.arange(1, self.map_width - 1)

        row_picks = np.random.choice(row_options, 2, replace=True)
        col_picks = np.random.choice(col_options, 2, replace=True)
        self.agent_loc = np.array([row_picks[0], col_picks[0]])
        # self.goal_loc = np.array([row_picks[1], col_picks[1]])
        self.goal_loc = np.array([self.map_height // 2, self.map_width // 2])

        # random multiple of 30 degrees
        self.agent_angle = np.random.choice(np.arange(0, 2 * np.pi, np.deg2rad(30)))

        # set map
        self.map = generate_gridworld(
            self.map_height, self.map_width, (self.goal_loc[0], self.goal_loc[1])
        )

        self._get_obs()
        return self.obs, {}

    def render(self):
        # plot the gridworld with the agent and goal
        state = {
            "agent_position": tuple(self.estimate_loc()),
            "agent_angle": self.agent_angle,
            "angles": self.angles,
            "dists": self.dists,
        }
        plot_gridworld(self.map, state, wait=self.render_time)

    def close(self):
        pass

    def _get_obs(self):

        num_scans = self.lidar_hz * self.dt
        angles = []
        dists = []
        for i in range(num_scans):
            angle, dist = get_lidar_scans(
                self.map, self.agent_loc, spread=(5, 7)
            )
            angles += [np.average(angle)]
            dists += [np.average(dist)]
        self.angles = np.array(angles)
        self.dists = np.array(dists)

        obs = {
            "agent_angle": np.array([self.agent_angle]),
            # "agent_loc": np.array(self.agent_loc),
            # "goal_loc": np.array(self.goal_loc),
            "angles": self.angles,
            "dists": self.dists,
        }

        self.obs = obs

    def estimate_loc(self):
        # estimate the location of the agent based on its angle and speed
        estimated_loc = np.array([int(self.agent_loc[0]), int(self.agent_loc[1])])
        return estimated_loc


# # drive around
# env = simEnv(map_height=10, map_width=20)
# env.reset()
# env.render()
# for i in range(100):
#     env.step(0)
#     env.render()
import stable_baselines3 as sb3
from env import simEnv

map_height = 7
map_width = 7
TIMESTEPS = 1000000
save_interval = 100000
env = simEnv(map_height=map_height, map_width=map_width)

# train agent
model = sb3.PPO("MultiInputPolicy", env, verbose=1)

model_name = "models/ppo_sim-" + str(map_height) + "-" + str(map_width)
for i in range(TIMESTEPS // save_interval):
    model.learn(total_timesteps=save_interval)
    model.save(model_name)
print(model_name)

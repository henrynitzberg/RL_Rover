import stable_baselines3 as sb3
from env import simEnv

map_height = 7
map_width = 7
model = sb3.PPO.load(f"models/ppo_sim-{map_height}-{map_width}")

env = simEnv(map_height=map_height, map_width=map_width, render_time=.1)

obs, info = env.reset()
done = False
env.render()
for i in range(100):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    env.render()
    if done or truncated:
        print(f"Step {i}: action={action}, reward={reward}, done={done}")
        obs, info = env.reset()
        done = False
        env.render()
import os
import matplotlib.pyplot as plt
import stable_baselines3 as sb3
from stable_baselines3.common.callbacks import BaseCallback
from env import simEnv

class RewardTrackingCallback(BaseCallback):
    def __init__(self):
        super().__init__()
        self.rewards = []
        self.episode_rewards = []
        self.episode_count = 0

    def _on_step(self) -> bool:
        # Collect rewards from environment
        if done := self.locals.get("dones"):
            for idx, d in enumerate(done):
                if d:
                    self.episode_rewards.append(self.locals["infos"][idx].get("episode", {}).get("r", 0))
                    self.episode_count += 1
        return True

    def _on_training_end(self) -> None:
        self.rewards = self.episode_rewards

# Configuration
map_height = 7
map_width = 7
TIMESTEPS = 100000
save_interval = 10000
env = simEnv(map_height=map_height, map_width=map_width)

# Setup and training
model = sb3.PPO("MultiInputPolicy", env, verbose=1)
callback = RewardTrackingCallback()

model_name = f"models/ppo_sim-{map_height}-{map_width}"
os.makedirs("models", exist_ok=True)

for i in range(TIMESTEPS // save_interval):
    model.learn(total_timesteps=save_interval, reset_num_timesteps=False, callback=callback)
    model.save(f"{model_name}_step_{(i + 1) * save_interval}")

# Plot rewards
if callback.rewards:
    plt.figure(figsize=(10, 5))
    plt.plot(callback.rewards, label="Episode Reward")
    plt.xlabel("Episodes")
    plt.ylabel("Reward")
    plt.title("Training Reward Over Time")
    plt.legend()
    plt.grid(True)
    plt.savefig("training_rewards.png")
    plt.show()

print(f"Model saved to: {model_name}_step_{TIMESTEPS}")
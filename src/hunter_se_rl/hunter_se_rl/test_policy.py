"""
Test trained TD3 policy.
"""

import argparse
import time
import numpy as np
from stable_baselines3 import TD3
from .hunter_se_env import HunterSEEnv


def main():
    parser = argparse.ArgumentParser(description='Test TD3 policy')
    parser.add_argument('--model', type=str, required=True)
    parser.add_argument('--episodes', type=int, default=10)
    args = parser.parse_args()

    env = HunterSEEnv(environment_dim=20)
    model = TD3.load(args.model)

    total_rewards, successes, collisions = [], 0, 0

    for ep in range(args.episodes):
        obs, info = env.reset()
        done, truncated, episode_reward, steps = False, False, 0, 0

        print(f"\nEpisode {ep + 1}/{args.episodes} - Goal: ({info['goal_x']:.2f}, {info['goal_y']:.2f})")

        while not done and not truncated:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            steps += 1

        total_rewards.append(episode_reward)
        if info.get('reached_goal'): successes += 1; print(f"  SUCCESS in {steps} steps")
        elif info.get('collision'): collisions += 1; print(f"  COLLISION at step {steps}")
        else: print(f"  TIMEOUT after {steps} steps")
        print(f"  Reward: {episode_reward:.2f}")

    print(f"\n{'='*50}\nSUMMARY\n{'='*50}")
    print(f"Success: {successes}/{args.episodes} ({100*successes/args.episodes:.1f}%)")
    print(f"Collision: {collisions}/{args.episodes} ({100*collisions/args.episodes:.1f}%)")
    print(f"Avg reward: {np.mean(total_rewards):.2f} (+/- {np.std(total_rewards):.2f})")

    env.close()


if __name__ == '__main__':
    main()
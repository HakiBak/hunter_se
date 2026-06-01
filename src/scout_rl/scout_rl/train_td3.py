"""
TD3 Training script using Stable Baselines3.
"""

import os
import argparse
import numpy as np

from stable_baselines3 import TD3
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, CallbackList
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.monitor import Monitor

from .scout_env import ScoutEnv


def main():
    parser = argparse.ArgumentParser(description='Train TD3 on Scout Mini')
    parser.add_argument('--timesteps', type=int, default=1000000)
    parser.add_argument('--load-model', type=str, default=None)
    parser.add_argument('--save-dir', type=str, default='./models')
    parser.add_argument('--log-dir', type=str, default='./logs')
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)

    print("Creating environment...")
    env = Monitor(ScoutEnv(environment_dim=20), args.log_dir)

    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    if args.load_model:
        print(f"Loading model from {args.load_model}")
        model = TD3.load(args.load_model, env=env)
        model.action_noise = action_noise
    else:
        print("Creating new TD3 model...")
        model = TD3(
            "MlpPolicy", env,
            learning_rate=3e-4, buffer_size=100000, learning_starts=10000,
            batch_size=256, tau=0.005, gamma=0.99,
            train_freq=(1, "step"), gradient_steps=1,
            action_noise=action_noise,
            policy_kwargs=dict(net_arch=[400, 300]),
            tensorboard_log=args.log_dir, verbose=1)

    callbacks = CallbackList([
        CheckpointCallback(save_freq=10000, save_path=args.save_dir, name_prefix='td3_scout'),
        EvalCallback(env, best_model_save_path=args.save_dir, log_path=args.log_dir,
                     eval_freq=5000, n_eval_episodes=5, deterministic=True)])

    print(f"Starting training for {args.timesteps} timesteps...")
    try:
        model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=True)
    except KeyboardInterrupt:
        print("\nTraining interrupted")

    model.save(os.path.join(args.save_dir, 'td3_final'))
    print(f"Model saved to {args.save_dir}/td3_final")
    env.close()


if __name__ == '__main__':
    main()

#     """
# TD3 Training with Parallel Environments for Scout Mini Robot.
# Uses SubprocVecEnv to run multiple Gazebo instances simultaneously.
# """

# import argparse
# from stable_baselines3 import TD3
# from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
# from stable_baselines3.common.noise import NormalActionNoise
# from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
# import numpy as np


# def make_env(env_id: int):
#     """
#     Factory function to create environment instances.
#     Each environment gets a unique ID for namespacing ROS topics.
#     """
#     def _init():
#         # Import here to ensure fresh ROS context per subprocess
#         from scout_rl.scout_env import ScoutEnv
        
#         # Each env needs unique ROS namespace to avoid topic conflicts
#         # This requires modifying ScoutEnv to accept namespace parameter
#         env = ScoutEnv(environment_dim=20, namespace=f"scout_{env_id}")
#         return env
#     return _init


# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--timesteps', type=int, default=500000)
#     parser.add_argument('--num_envs', type=int, default=4)  # Number of parallel envs
#     args = parser.parse_args()

#     # Create vectorized environment with multiple parallel instances
#     # SubprocVecEnv runs each env in separate process - ideal for Gazebo
#     env = SubprocVecEnv([make_env(i) for i in range(args.num_envs)])
    
#     # Wrap with monitor for logging
#     env = VecMonitor(env)

#     # Action noise for exploration (applied to all envs)
#     n_actions = env.action_space.shape[-1]
#     action_noise = NormalActionNoise(
#         mean=np.zeros(n_actions),
#         sigma=0.1 * np.ones(n_actions)
#     )

#     # TD3 model - batch_size should be multiple of num_envs
#     model = TD3(
#         "MlpPolicy",
#         env,
#         learning_rate=3e-4,
#         buffer_size=100000,
#         batch_size=256,
#         learning_starts=10000,
#         train_freq=(1, "episode"),
#         gradient_steps=-1,
#         tau=0.005,
#         gamma=0.99,
#         action_noise=action_noise,
#         policy_kwargs=dict(net_arch=[400, 300]),
#         tensorboard_log="./logs",
#         verbose=1
#     )

#     # Callbacks
#     checkpoint_callback = CheckpointCallback(
#         save_freq=10000 // args.num_envs,  # Adjust for parallel envs
#         save_path="./checkpoints",
#         name_prefix="td3_scout"
#     )

#     # Train
#     model.learn(
#         total_timesteps=args.timesteps,
#         callback=[checkpoint_callback],
#         progress_bar=True
#     )

#     model.save("td3_final")
#     env.close()


# if __name__ == "__main__":
#     main()

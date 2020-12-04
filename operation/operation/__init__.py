from gym.envs.registration import register

register(
    id='operation-v0',
    entry_point='operation.envs:OperationEnv',
    max_episode_steps=1000,
    reward_threshold=5.0,
)
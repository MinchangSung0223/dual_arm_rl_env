from gym.envs.registration import register
register(
    id='DualArmEnv-v0', 
    entry_point='dual_arm_env.envs:DualArmEnv'
)


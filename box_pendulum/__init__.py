from gym.envs.registration import register
register(
    id='CubeStand-v0',
    entry_point='box_pendulum.envs:CubeStandEnv'
)

register(
    id='Cube3DStand-v0',
    entry_point='box_pendulum.envs:Cube3DStandEnv'
)
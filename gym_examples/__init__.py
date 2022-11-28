from gym.envs.registration import register

register(
    id="gym_examples/GridWorld-v0",
    entry_point="gym_examples.envs:GridWorldEnv",
)

register(
    id="gym_examples/RotaryInvertedPendulum-v0",
    entry_point="gym_examples.envs:RotaryInvertedPendulumEnv",
)

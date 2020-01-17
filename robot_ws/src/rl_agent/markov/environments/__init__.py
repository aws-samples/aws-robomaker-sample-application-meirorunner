from gym.envs.registration import register

MAX_STEPS = 1000

register(
    id='RoboMaker-ObjectTracker-v0',
    entry_point='markov.environments.object_tracker_env:TurtleBot3ObjectTrackerAndFollowerDiscreteEnv',
    max_episode_steps = MAX_STEPS,
    reward_threshold = 200
)

register(
    id='RoboMaker-MeiroRunner-v0',
    entry_point='markov.environments.meiro_runner_env:TurtleBot3MeiroRunnerDiscreteEnv',
    max_episode_steps = MAX_STEPS,
    reward_threshold = 200
)

from gymnasium.envs.registration import register

from .sailboat_lsa import SailboatLSAEnv
from .env import *

env_by_name = {
    "SailboatLSAEnv-v0": SailboatLSAEnv,
}

for name, env in env_by_name.items():
    register(
        id=name,
        entry_point=f"sailboat_gym.sailboat_gym.envs:{env.__name__}",
    )

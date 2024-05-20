

import os

MINI_GYM_ROOT_DIR = os.path.join(
    os.path.dirname(
        os.path.dirname(
            os.path.realpath(__file__))),
    '..')
print(MINI_GYM_ROOT_DIR)
MINI_GYM_ENVS_DIR = os.path.join(MINI_GYM_ROOT_DIR, 'go_gym', 'envs')

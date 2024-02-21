import select
import numpy as np
import yaml

from Go2Py.sim.isaacsim.lcm_types.unitree_lowlevel import UnitreeLowCommand, UnitreeLowState


def load_config(file_path):
    with open(file_path, "r") as file:
        config = yaml.safe_load(file)
    return config

class NumpyMemMapDataPipe:
    def __init__(self, channel_name, force=False, dtype="uint8", shape=(640, 480, 3)):
        self.channel_name = channel_name
        self.force = force
        self.dtype = dtype
        self.shape = shape
        if force:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="w+", shape=shape
            )
        else:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="r+", shape=shape
            )

    def write(self, data, match_length=True):
        if match_length:
            self.shm[: data.shape[0], ...] = data
        else:
            assert (
                data.shape == self.shape
            ), "The data and the shape of the shared memory must match"
            self.shm[:] = data

    def read(self):
        return self.shm.copy()


class simulationManager:
    def __init__(self, robot, lcm_server, default_cmd, physics_dt, lcm_timeout=0.01):
        self.robot = robot
        self.lcm_server = lcm_server
        self.missed_ticks = 0
        self.default_cmd = default_cmd
        self.robot.initialize()
        self.reset_required = True
        self.lcm_timeout = lcm_timeout
        self.physics_dt = physics_dt
        self.robot.setCommands(self.default_cmd)

    def step(self, timestamp):
        # Read the robot's state and send it to the LCM client
        state = self.robot.readStates()
        state.stamp = timestamp
        self.lcm_server.sendStates(state)
        # Wait for a response from the LCM client timeout=0.1 second
        lcm_cmd = self.lcm_server.getCommands(timeout=self.lcm_timeout)
        if lcm_cmd is not None:
            self.missed_ticks = 0
            # Reset the robot if the communication has been off for too long
            if self.reset_required:
                self.robot.initialize()
                self.robot.setCommands(self.default_cmd)
                self.reset_required = False
            self.robot.setCommands(lcm_cmd)
        else:
            self.missed_ticks += 1
        if self.missed_ticks > 0.2 / (
            self.lcm_timeout + self.physics_dt
        ):  # stopped for more than a second?
            self.reset_required = True
            self.robot.setCommands(self.default_cmd)

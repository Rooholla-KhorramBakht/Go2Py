# launch Isaac Sim before any other imports
# default first two lines in any standalone application
import pickle
import sys
import time

import numpy as np
from omni.isaac.kit import SimulationApp

# sys.path.append("/home/vr-station/projects/lcm/build/python")


simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

# import cv2
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.sensor import RotatingLidarPhysX

import Go2Py
from Go2Py.sim.isaacsim.lcm_types.unitree_lowlevel import UnitreeLowCommand
from Go2Py.sim.isaacsim.go2 import UnitreeGo2
from Go2Py.sim.isaacsim.utils import AnnotatorManager
from Go2Py.sim.utils import (
    NumpyMemMapDataPipe,
    load_config,
    simulationManager,
)

cfg = load_config(Go2Py.GO2_ISAACSIM_CFG_PATH)
robots = cfg["robots"]
cameras = cfg["cameras"]
env_cfg = cfg["environment"]

PHYSICS_DT = 1 / 200
RENDERING_DT = 1 / 200

# create the world
world = World(
    physics_dt=cfg["environment"]["simulation_dt"],
    rendering_dt=cfg["environment"]["rendering_dt"],
)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets folder")

prim = get_prim_at_path(env_cfg["prim_path"])
if not prim.IsValid():
    prim = define_prim(env_cfg["prim_path"], "Xform")
    asset_path = (
        assets_root_path + env_cfg["usd_path"]
        if env_cfg["buildin"] is True
        else env_cfg["usd_path"]
    )
    prim.GetReferences().AddReference(asset_path)

breakpoint()
go2 = world.scene.add(
    UnitreeGo2(
        prim_path=robots[0]["prim_path"],
        usd_path=(robots[0]["usd_path"] if robots[0]["usd_path"] != "" else None),
        name=robots[0]["name"],
        position=np.array(robots[0]["position"]),
        physics_dt=cfg["environment"]["simulation_dt"],
    )
)

# # Add Lidar
# lidar = world.scene.add(
#     RotatingLidarPhysX(
#         prim_path="/World/Env/GO2/imu_link/lidar",
#         name="lidar",
#         translation=[0.16, 0.0, 0.14],
#         orientation=[0.0, 0.0, 0.0, 1.0],
#     )
# )

# lidar.add_depth_data_to_frame()
# lidar.add_point_cloud_data_to_frame()
# lidar.set_rotation_frequency(0)
# lidar.set_resolution([0.4, 2])
# # lidar.enable_visualization()
# lidar.prim.GetAttribute("highLod").Set(True)
# lidar.prim.GetAttribute("highLod").Set(True)

# lidar_data_pipe = NumpyMemMapDataPipe(
#     "lidar_data_pipe", force=True, dtype="float32", shape=(900, 16, 3)
# )

world.reset()
go2.initialize()
breakpoint()
# Add cameras
print("Adding cameras")
ann = AnnotatorManager(world)

for camera in cameras:
    ann.registerCamera(
        camera["prim_path"],
        camera["name"],
        translation=camera["translation"],  # xyz
        orientation=camera["orientation"],  # xyzw
        resolution=camera["resolution"],
    )
    for type in camera["type"]:
        ann.registerAnnotator(type, camera["name"])

    ann.setClippingRange(camera["name"], 0.2, 1000000.0)
    ann.setFocalLength(camera["name"], 28)

# Add the shared memory data channels for image and LiDAR data
print("Creating shared memory data pipes")
camera_pipes = {}
for camera in cameras:
    for type in camera["type"]:
        if type == "pointcloud":
            pipe = NumpyMemMapDataPipe(
                camera["name"] + "_" + type,
                force=True,
                dtype="float32",
                shape=(camera["resolution"][1] * camera["resolution"][0], 3),
            )
            camera_pipes[camera["name"] + "_" + type] = pipe
        elif type == "rgb":
            pipe = NumpyMemMapDataPipe(
                camera["name"] + "_" + type,
                force=True,
                dtype="uint8",
                shape=(camera["resolution"][1], camera["resolution"][0], 4),
            )
            camera_pipes[camera["name"] + "_" + type] = pipe
        elif type == "distance_to_camera":
            pipe = NumpyMemMapDataPipe(
                camera["name"] + "_" + type,
                force=True,
                dtype="uint8",
                shape=(camera["resolution"][1], camera["resolution"][0]),
            )
            camera_pipes[camera["name"] + "_" + type] = pipe

# Store simulation hyperparamters in shared memory
print("Storing simulation hyperparamters in shared memory")

meta_data = {
    "camera_names": [camera["name"] for camera in cameras],
    "camera_types": [camera["type"] for camera in cameras],
    "camera_resolutions": [camera["resolution"] for camera in cameras],
    "camera_intrinsics": [
        ann.getCameraIntrinsics(camera["name"]) for camera in cameras
    ],
    "camera_extrinsics": [
        ann.getCameraExtrinsics(camera["name"]) for camera in cameras
    ],
}
with open("/dev/shm/fr3_sim_meta_data.pkl", "wb") as f:
    pickle.dump(meta_data, f)

lcm_server = LCMBridgeServer(robot_name="b1")
cmd_stamp = time.time()
cmd_stamp_old = cmd_stamp

cmd = UnitreeLowCommand()
cmd.kd = 12 * [2.5]
cmd.kp = 12 * [100]
cmd.dq_des = 12 * [0]
cmd.q_des = b1.init_joint_pos

# sim_manager = simulationManager(
#     robot=go2,
#     lcm_server=lcm_server,
#     default_cmd=cmd,
#     physics_dt=PHYSICS_DT,
#     lcm_timeout=1e-4,
# )
counter = 0
while simulation_app.is_running():
    # sim_manager.step(counter*PHYSICS_DT)
    # Step the world with rendering 50 times per second
    # sim_manager.step(counter * PHYSICS_DT)
    if counter % 4 == 0:
        world.step(render=True)
        pc = lidar.get_current_frame()["point_cloud"]
        lidar_data_pipe.write(pc, match_length=True)

        # Push the sensor data to the shared memory pipes
        for camera in cameras:
            for type in camera["type"]:
                data = ann.getData(f"{camera['name']}:{type}")
                if type == "pointcloud":
                    payload = data["data"]
                    if payload.shape[0]:
                        camera_pipes[camera["name"] + "_" + type].write(
                            np.zeros(
                                (camera["resolution"][1] * camera["resolution"][0], 3)
                            ),
                            match_length=True,
                        )
                        camera_pipes[camera["name"] + "_" + type].write(
                            payload, match_length=True
                        )
                else:
                    if data.shape[0]:
                        camera_pipes[camera["name"] + "_" + type].write(
                            data, match_length=False
                        )

    else:
        world.step(render=False)

    counter += 1
simulation_app.close()  # close Isaac Sim

import os
import torch
import numpy as np
from omni.isaac.kit import SimulationApp

# Simple example showing how to start and stop the helper
simulation_app = SimulationApp({"headless": True})

### Perform any omniverse imports here after the helper loads ###
from omni.isaac.core import World
from omni.isaac.cloner import GridCloner
import omni.isaac.core.utils.stage as stage_utils
from pxr import UsdGeom

from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.torch.rotations import *
from omni.isaac.sensor import IMUSensor

from isaacsim.iniedo.robots.articulations.husky import Husky
from isaacsim.iniedo.robots.articulations.views.husky_view import HuskyView

env_zero_path = "/World/envs/env_0"
num_envs = 4

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# test the actual calculation of differential drive
wheel_radius = 0.03
wheel_base = 0.1125
controller = DifferentialController("test_controller", wheel_radius, wheel_base)
linear_speed = 0.3
angular_speed = 1.0
command = [linear_speed, angular_speed]
actions = controller.forward(command)

##################### Create ENV0 #####################

husky = Husky(
    prim_path=env_zero_path + "/Husky", name="Husky", translation=torch.tensor([0, 0, 0.5])
)
my_world.scene.add(
    IMUSensor(
        prim_path=env_zero_path+"/Husky/imu_link/Imu_Sensor",
        name="imu",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )
)
# rovers = HuskyView(prim_paths_expr="/World/envs/env_0/Husky", name="husky_view")

##################### Create clone environments #####################

# clone the environment (num_envs)
cloner = GridCloner(spacing=1.5)
cloner.define_base_env(env_zero_path)
UsdGeom.Xform.Define(stage_utils.get_current_stage(), env_zero_path)
cloner.clone(source_prim_path=env_zero_path, prim_paths=cloner.generate_paths("/World/envs/env", num_envs))

# pos = np.array([[0,0,0]])
# pos, _ = cloner.get_clone_transforms(num_envs)
# env_pos = torch.tensor(np.array(pos), device="cuda:0", dtype=torch.float)

rovers = HuskyView(prim_paths_expr="/World/envs/.*/Husky", name="husky_view")

# Add IMU sensor
imu_sensors = []
for i in range(num_envs):
    imu_sensors.append(IMUSensor(prim_path="/World/envs/env_"+str(i)+"/Husky/imu_link/Imu_Sensor"))

##################### Set articulations #####################

# from controller import Controller
# from control_config import control
# cntrl = Controller(control,"cuda:0")


# leos = ArticulationView(prim_paths_expr="/World/envs/env_[0-4]/Leo",name="leos_view")
# actions = ArticulationAction(joint_velocities=np.array([2,2,-2,-2]))
# actions = ArticulationAction(joint_velocities=np.array([[2,2,-2,-2],[2,2,-2,-2],[2,2,-2,-2],[2,2,-2,-2]]))
# test_targets = np.array([[2,2,-2,-2],[2,2,-2,-2],[2,2,-2,-2],[2,2,-2,-2]])
test_targets = np.array([[2,-2,2,-2],[2,-2,2,-2],[2,-2,2,-2],[2,-2,2,-2]])
# test_targets = np.array([2,-2,2,-2])
# targets = torch.tensor([[0,0,2,0],[0,0,2,0],[0,0,2,0],[0,0,2,0]],device="cuda:0")
# forces = np.ones([4,4])*(1.5+(0.03*9.8066)/4)

# obs_buf = torch.zeros((num_envs, 13), device="cuda:0", dtype=torch.float)
# all_indices = torch.arange(num_envs, dtype=torch.int32, device="cuda:0")

##################### Begin sim #####################


count = 0

my_world.reset()
# controller.reset()
rovers.initialize()
while simulation_app.is_running():
    # simulation_app.update()  # Render a single frame
    my_world.step(render=True)
    print("\n\n##########################")
    print(imu_sensors[2].get_current_frame())
    print(imu_sensors[3].get_current_frame())

    if count >5000:
        break
    count = count + 1

    # copters.set_joint_efforts(efforts=forces)
    rovers.set_joint_velocity_targets(velocities=test_targets)
    # leo.apply_wheel_actions(actions)
    # leos.set_joint_velocities(actions)
simulation_app.close()  # Cleanup application
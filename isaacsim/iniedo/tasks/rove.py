import math

import numpy as np
import torch
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.sensor import IMUSensor

from omniisaacgymenvs.tasks.base.rl_task import RLTask
from isaacsim.iniedo.robots.articulations.husky import Husky
from isaacsim.iniedo.robots.articulations.views.husky_view import HuskyView

class RoveTask(RLTask):
    def __init__(self, name, sim_config, env, offset=None) -> None:

        self.update_config(sim_config)
        self._num_observations = 6
        self._num_actions = 4

        RLTask.__init__(self, name=name, env=env, offset=offset)
        return

    def update_config(self, sim_config):
        self._sim_config = sim_config
        self._cfg = sim_config.config
        self._task_cfg = sim_config.task_config

        self._num_envs = self._task_cfg["env"]["numEnvs"]
        self._env_spacing = self._task_cfg["env"]["envSpacing"]
        self._max_episode_length = self._task_cfg["env"]["episodeLength"]

        self._reset_dist = self._task_cfg["env"]["resetDist"]
        self._max_push_effort = self._task_cfg["env"]["maxEffort"]

        self.dt = self._task_cfg["sim"]["dt"]

    def set_up_scene(self, scene) -> None:
        # Get robot assets and apply articulation settings
        self.get_husky()
        # self.get_copter()

        # Add IMU to husky imu link
        # zero_env_imu_prim_path = self.default_zero_env_path+"/Husky/imu_link/Imu_Sensor"
        zero_env_imu_prim_path = self.default_zero_env_path+"/Husky/base_link/Imu_Sensor"
        scene.add(
            IMUSensor(prim_path=zero_env_imu_prim_path, name="imu", frequency=1/self.dt, translation=np.array([0, 0, 0]),
            )
        )
        # Setup parallel envs
        RLTask.set_up_scene(self, scene)

        # Setup and add articulation views to scene
        self._huskies = HuskyView(prim_paths_expr="/World/envs/.*/Husky", name="husky_view")
        scene.add(self._huskies)
        # for idx in range(len(self._huskies.physics_wheels)):
        #     scene.add(self._huskies.physics_wheels[idx])

        # Add IMU sensor
        self.imu_sensors = []
        for idx in range(self._num_envs):
            self.imu_sensors.append(IMUSensor(prim_path="/World/envs/env_"+str(idx)+"/Husky/imu_link/Imu_Sensor"))

        return

    def get_husky(self):
        husky = Husky(prim_path=self.default_zero_env_path + "/Husky", name="Husky", translation=torch.tensor([0, 0, 0.5]))
        self._sim_config.apply_articulation_settings(
            "Rove", get_prim_at_path(husky.prim_path), self._sim_config.parse_actor_config("Rove")
        )

    def post_reset(self):
        # implement any logic required for simulation on-start here
        pass

    def pre_physics_step(self, actions):
        # implement logic to be performed before physics steps
        # self.perform_reset()
        # self.apply_action(actions)
        # print("\n\n##########################")
        # print(self.imu_sensors[2].get_current_frame())
        # print(self.imu_sensors[3].get_current_frame())
        pass

    def get_observations(self) -> dict:
        # implement logic to retrieve observation states
        # self.obs_buf = self.compute_observations()
        return self.obs_buf

    def calculate_metrics(self) -> None:
        # implement logic to compute rewards
        # self.rew_buf = self.compute_rewards()
        pass

    def is_done(self) -> None:
        # implement logic to update dones/reset buffer
        # self.reset_buf = self.compute_resets()
        pass

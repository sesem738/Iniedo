IsaacSim Setup Instructions
---

- Install IsaacSim app using the Omniverse Launcher
- For installing python conda environmentrefer [Advanced: Running with Anaconda](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#advanced-running-with-anaconda)
- Test example :`python standalone_examples/api/omni.isaac.sensor/imu_sensor.py` from IsaacSim install directory
- If you see `libGL error: MESA-LOADER` issues take a look at this [potential fix](https://github.com/ContinuumIO/anaconda-issues/issues/12889#issuecomment-1742063241)
- Clone [OmniIsaacGymEnvs](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs) and follow setup instructions
- To use IMU sensors with gym add `"omni.isaac.sensor" = {}` dependency towards the end of `/path/to/isaac_sim-2023.*/apps/omni.isaac.sim.python.gym.kit` and `/path/to/isaac_sim-2023.*/apps/omni.isaac.sim.python.gym.headless.kit`
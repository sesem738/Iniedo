from typing import Optional

import carb
import numpy as np
import torch
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

class Leo(Robot):
    def __init__(
        self,
        prim_path: str,
        name: Optional[str] = "leo",
        usd_path: Optional[str] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.array] = None,
    ) -> None:

        self._usd_path = usd_path
        self._name = name

        if self._usd_path is None:
            # assets_root_path = get_assets_root_path()
            assets_root_path = "/home/sagar/ws/Iniedo/isaacsim/assets"
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
            self._usd_path = assets_root_path + "/leo_description/leo/leo.usd"
        
        add_reference_to_stage(self._usd_path,prim_path)
        scale = torch.tensor([1,1,1])

        super().__init__(prim_path=prim_path, name=name, translation=translation, orientation=orientation, scale=scale)

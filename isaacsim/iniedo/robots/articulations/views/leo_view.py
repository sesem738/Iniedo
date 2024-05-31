from typing import Optional

from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView


class LeoView(ArticulationView):
    def __init__(self, prim_paths_expr: str, name: Optional[str] = "Leo") -> None:
        """[summary]"""

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
        )

        wheels_dict = {
            "m_fl_view": 'rocker_L_link/wheel_FL_joint',
            "m_rl_view": 'rocker_L_link/wheel_RL_joint',
            "m_fr_view": 'rocker_R_link/wheel_FR_joint',
            "m_rr_view": 'rocker_R_link/wheel_RR_joint'}

        self.physics_wheels = [
            RigidPrimView(prim_paths_expr=f"/World/envs/.*/"+name+"/"+value, name=key)
            for (key, value) in wheels_dict.items()
        ]

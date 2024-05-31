from typing import Optional

from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView


class HuskyView(ArticulationView):
    def __init__(self, prim_paths_expr: str, name: Optional[str] = "HuskyView") -> None:
        """[summary]"""

        super().__init__(
            prim_paths_expr=prim_paths_expr,
            name=name,
            reset_xform_properties=False,
        )

        wheels_dict = {
            "m_fl_view": 'base_link/front_left_wheel',  # 0
            "m_rl_view": 'base_link/rear_left_wheel',   # 1
            "m_fr_view": 'base_link/front_right_wheel', # 2
            "m_rr_view": 'base_link/rear_right_wheel'   # 3
        }
        
        # self.physics_wheels = [
        #     RigidPrimView(prim_paths_expr=f"/World/envs/.*/Husky/"+value, name=key, reset_xform_properties=False)
        #     for (key, value) in wheels_dict.items()
        # ]

        return

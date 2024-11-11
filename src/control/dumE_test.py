import numpy as np
import roboticstoolbox as rtb
import os
from roboticstoolbox.robot.ERobot import ERobot
from ament_index_python.packages import get_package_share_directory


class dumE(ERobot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read("dku_robotarm.urdf", tld=get_package_share_directory("dku_robotarm")+'/urdf')
        super().__init__(
            links,
            name=name,
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            manufacturer="dumE",
            gripper_links=links[10]
        )

        self.addconfiguration(
            "qr", np.array([0, 0, 0, 0, 0, 0])
        )

if __name__ == "__main__":  # pragma nocover

    robot = dumE()
    ets = robot.ets()
    solver = rtb.IK_NR()
    Tep = robot.fkine([0, 0, 0, 0, 0, 0])
    print(ets, Tep)
    print(solver.solve(ets, Tep))
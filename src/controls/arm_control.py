from wpilib import Joystick, DriverStation

from subsystems.arm import Arm

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from config.constants import ArmConst


class Priority:
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


class ArmControl(AdaptiveComponent):
    def __init__(self, robot: "AdaptiveRobot", arm: Arm, controller: Joystick) -> None:
        super().__init__(robot)

        self.arm = arm
        
        self.controller = controller

    def execute(self) -> None:
        """
        Declare when the arm should move depending on user input.
        This is automatically called by the scheduler each iteration.
        """
        if not DriverStation.isTeleopEnabled():
            return
        
        if self.controller.getRawButton(1):
            self.arm.request_move_arm(ArmConst.APPLIED_PCT_OUT, Priority.TELEOP, "teleop")
        if self.controller.getRawButton(2):
            self.arm.request_move_arm(-ArmConst.APPLIED_PCT_OUT, Priority.TELEOP, "teleop")

from wpilib import Joystick, DriverStation

from components.arm import Arm

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from config.constants import ArmConst

class ArmControl(AdaptiveComponent):
    def __init__(self, robot: "AdaptiveRobot", arm: Arm, controller: Joystick) -> None:
        super().__init__(robot)

        self.arm = arm
        
        self.controller = controller

    def execute(self) -> None:
        """
        Declare when the arm should move depending on user input.
        This is automatically called by AdaptiveRobot.
        """
        # No commands can be made outside of teleop mode.
        if not DriverStation.isTeleopEnabled():
            self.arm.request_stop()
            return
        
        if self.controller.getRawButton(1):
            self.arm.request_move_arm(ArmConst.APPLIED_PCT_OUT)
        if self.controller.getRawButton(2):
            self.arm.request_move_arm(-ArmConst.APPLIED_PCT_OUT)

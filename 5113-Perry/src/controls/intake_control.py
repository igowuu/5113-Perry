from wpilib import Joystick, DriverStation

from components.intake import Intake

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from config.constants import ArmConst

class IntakeControl(AdaptiveComponent):
    def __init__(self, robot: "AdaptiveRobot", intake: Intake, controller: Joystick) -> None:
        super().__init__(robot)

        self.intake = intake
        
        self.controller = controller

    def execute(self) -> None:
        """
        Declare when the arm should move depending on user input.
        This is automatically called by AdaptiveRobot.
        """
        # No commands can be made outside of teleop mode.
        if not DriverStation.isTeleopEnabled():
            self.intake.request_stop()
            return
        
        if self.controller.getRawButton(3):
            self.intake.request_move_intake(ArmConst.APPLIED_PCT_OUT)
        if self.controller.getRawButton(4):
            self.intake.request_move_intake(-ArmConst.APPLIED_PCT_OUT)

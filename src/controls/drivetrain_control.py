from wpilib import DriverStation, Joystick

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from subsystems.drivetrain import Drivetrain


class Priority:
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


class DrivetrainControl(AdaptiveComponent):
    def __init__(
        self, 
        robot: AdaptiveRobot, 
        drivetrain: Drivetrain, 
        lstick: Joystick, 
        rstick: Joystick
    ) -> None:
        super().__init__(robot)

        self.drivetrain = drivetrain

        self.lstick = lstick
        self.rstick = rstick

    def execute(self) -> None:
        """
        Declare when the drivetrain should move based on user input.
        This method is automatically called each iteration by the scheduler.
        """
        if not DriverStation.isTeleopEnabled():
            return
        
        forward_speed_pct = -self.lstick.getY()
        angular_speed_pct = self.rstick.getX()
        
        self.drivetrain.request_forward_percent(forward_speed_pct, Priority.TELEOP, "teleop")
        self.drivetrain.request_angular_percent(angular_speed_pct, Priority.TELEOP, "teleop")

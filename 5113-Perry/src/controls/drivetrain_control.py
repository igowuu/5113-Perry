from wpilib import DriverStation, Joystick

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from components.drivetrain import Drivetrain, DrivePriority


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
        if not DriverStation.isTeleopEnabled():
            return
        
        forward_speed_pct = -self.lstick.getY()
        rotation_speed_pct = self.rstick.getX()
        
        self.drivetrain.request_forward_percent(forward_speed_pct, DrivePriority.TELEOP, "teleop")
        self.drivetrain.request_rotation_percent(rotation_speed_pct, DrivePriority.TELEOP, "teleop")

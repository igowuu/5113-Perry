# Perry - 5113 FRC code
# Copyright (c) 2026 Jacob Taylor (igowu) <https://github.com/igowuu>
# Code from: <https://github.com/igowuu/AdaptiveRobot.git>

# Licensed under the MIT License.
# See https://opensource.org/licenses/MIT for details.

from wpilib import Joystick

from subsystems.drivetrain import Drivetrain
from subsystems.arm import Arm
from subsystems.intake import Intake

from controls.drivetrain_control import DrivetrainControl
from controls.arm_control import ArmControl
from controls.intake_control import IntakeControl

from utils.dashboard import Dashboard

from autonomous.routines.test_drive import TestDriveRoutine
from autonomous.manager import AutoManager

from adaptive_robot.adaptive_robot import AdaptiveRobot


class Priority:
    """
    This holds the desired priorities for each file that submits requests to subsystems.

    In this case, SAFETY requests will override all other requests. AUTO requests will
    override TELEOP requests.
    """
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


class Perry(AdaptiveRobot):
    """
    robot.py initializes and orchestrates all files.
    """

    def onRobotInit(self) -> None:
        self.lstick = Joystick(0)
        self.rstick = Joystick(1)

        self.drivetrain = Drivetrain(self)
        self.drive_control = DrivetrainControl(self, self.drivetrain, self.lstick, self.rstick)
        
        self.arm = Arm(self)
        self.arm_control = ArmControl(self, self.arm, self.lstick)
        
        self.intake = Intake(self)
        self.intake_control = IntakeControl(self, self.intake, self.lstick)

        self.dashboard = Dashboard(self, self.drivetrain)

        self.auto_manager = AutoManager(self)

    def disabledPeriodic(self) -> None:
        """
        Stop all subsystems when disabled.
        """
        self.drivetrain.request_stop(Priority.SAFETY, "safety")
        self.arm.request_stop(Priority.SAFETY, "safety")
        self.intake.request_stop(Priority.SAFETY, "safety")

    def autonomousInit(self) -> None:
        # This is a call to the test autonomous routine.
        # In the near future, this will support choosing a routine through NetworkTables.
        routine = TestDriveRoutine(self.drivetrain)
        self.auto_manager.start(routine)

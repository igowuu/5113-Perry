from autonomous.routines.routine_base import SequentialRoutine
from autonomous.steps.drive_sysid import DriveLeftSysId, DriveRightSysId

from subsystems.drivetrain import Drivetrain


class SysIdDrive(SequentialRoutine):
    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__([
            DriveLeftSysId(drivetrain),
            DriveRightSysId(drivetrain)
        ])

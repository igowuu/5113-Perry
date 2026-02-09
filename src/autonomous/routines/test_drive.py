from subsystems.drivetrain import Drivetrain

from autonomous.steps.follow_trajectory import DriveTrajectoryStep
from autonomous.routines.routine_base import SequentialRoutine


class TestDriveRoutine(SequentialRoutine):
    """
    Test drive to showcase the drive_to_odometry step.
    """
    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__([
            DriveTrajectoryStep(
                drivetrain=drivetrain
            )
        ])

import wpilib
from wpilib import Timer, Field2d
import wpilib.shuffleboard
from wpimath.controller import LTVUnicycleController
from wpimath.units import seconds

import choreo
from choreo.trajectory import DifferentialSample

from subsystems.drivetrain import Drivetrain
from autonomous.steps.step_base import Step

from config.constants import RobotConst, DrivetrainPID


class Priority:
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


def sample_at(timestamp: seconds, samples: list[DifferentialSample]) -> DifferentialSample | None:
    """
    Returns the sample at the closest time to the provided timestamp.
    This is a temporary method that exists solely because of a bug within choreo.
    """
    # TODO: Binary search instead of basic iteration (efficiency concern)
    closest_sample = None
    min_diff = float('inf') 

    for sample in samples:
        # Calculate the time difference
        time_diff = abs(sample.timestamp - timestamp)
        
        if time_diff < min_diff:
            min_diff = time_diff
            closest_sample = sample

    return closest_sample


class DriveTrajectoryStep(Step):
    """
    Follows a choreo trajectory using an LTV controller.
    """
    def __init__(
        self,
        drivetrain: Drivetrain
    ) -> None:
        self.drivetrain = drivetrain

        self.controller = LTVUnicycleController(
             DrivetrainPID.B,
             DrivetrainPID.ZETA
        )

        self.timer = Timer()
        self.choreo_trajectory = choreo.load_differential_trajectory("TestPath")
        self.samples = self.choreo_trajectory.get_samples()

        self.actual_field = Field2d()
        self.trajectory_field = Field2d()
        
        wpilib.shuffleboard.Shuffleboard.getTab("Field").add("ActualField2d", self.actual_field)
        wpilib.shuffleboard.Shuffleboard.getTab("Field").add("TrajectoryField2d", self.trajectory_field)

    def start(self) -> None:
        self.timer.reset()
        self.timer.start()

    def update(self) -> None:
        current_pose = self.drivetrain.get_pose()
        self.actual_field.setRobotPose(current_pose)

        t = self.timer.get()
        sample = sample_at(t, self.samples)
        
        if not sample:
            return

        desired_pose = sample.get_pose()

        self.trajectory_field.setRobotPose(desired_pose)

        desired_chassis = sample.get_chassis_speeds()

        chassis_speeds = self.controller.calculate(
            currentPose=current_pose,
            poseRef=desired_pose,
            linearVelocityRef=desired_chassis.vx,
            angularVelocityRef=desired_chassis.omega
        )

        self.drivetrain.request_forward_percent(
            forward_speed=chassis_speeds.vx / RobotConst.MAX_SPEED_MPS, 
            priority=Priority.AUTO, 
            source="auto"
        )
        self.drivetrain.request_angular_percent(
            angular_speed=chassis_speeds.omega / RobotConst.MAX_SPEED_RADPS, 
            priority=Priority.AUTO, 
            source="auto"
        )

    def is_finished(self) -> bool:
        return self.timer.get() >= self.choreo_trajectory.get_total_time()

    def stop(self) -> None:
        self.drivetrain.request_stop(Priority.SAFETY, "safety")


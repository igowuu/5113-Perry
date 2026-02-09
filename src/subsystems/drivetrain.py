from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode
from navx import AHRS

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveKinematics, ChassisSpeeds
from wpimath.filter import SlewRateLimiter
from wpimath.units import percent, volts, meters, meters_per_second

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent
from adaptive_robot.requests import AxisController

from utils.math_utils import (
    clamp, 
    rotations_to_meters, 
    rps_to_mps,
    rpm_to_rps, 
    degrees_to_radians
)
from config.constants import RobotConst, DrivetrainConst, DrivetrainPID, DrivetrainFF


class Drivetrain(AdaptiveComponent):
    """
    This class declares the drivetrain to handle all linear and angular motion of our bot.
    """
    def __init__(self, robot: "AdaptiveRobot") -> None:
        super().__init__(robot)

        self.front_left_motor = SparkMax(11, SparkMax.MotorType.kBrushless)
        self.front_right_motor = SparkMax(13, SparkMax.MotorType.kBrushless)

        self.front_left_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            .voltageCompensation(RobotConst.NOMINAL_VOLTAGE),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.front_right_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake).inverted(True)
            .voltageCompensation(RobotConst.NOMINAL_VOLTAGE),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.front_left_encoder = self.front_left_motor.getEncoder()
        self.front_right_encoder = self.front_right_motor.getEncoder()
        
        self.front_left_encoder.setPosition(0)
        self.front_right_encoder.setPosition(0)

        self.gyro = AHRS.create_spi()
        self.gyro.reset()

        self.odometry = DifferentialDriveOdometry(
            self.get_heading(),
            self.get_left_distance(),
            self.get_right_distance()
        )
        
        self.left_pid = self.tunablePID(
            kp=DrivetrainPID.LEFT_KP,
            ki=DrivetrainPID.LEFT_KI,
            kd=DrivetrainPID.LEFT_KD,
            directory="Tunables/Drive/LeftPIDController"
        )
        self.right_pid = self.tunablePID(
            kp=DrivetrainPID.RIGHT_KP,
            ki=DrivetrainPID.RIGHT_KI,
            kd=DrivetrainPID.RIGHT_KD,
            directory="Tunables/Drive/RightPIDController"
        )

        self.left_ff = SimpleMotorFeedforwardMeters(
            kS=DrivetrainFF.LEFT_KS,
            kV=DrivetrainFF.LEFT_KV,
            kA=DrivetrainFF.LEFT_KA
        )
        self.right_ff = SimpleMotorFeedforwardMeters(
            kS=DrivetrainFF.RIGHT_KS,
            kV=DrivetrainFF.RIGHT_KV,
            kA=DrivetrainFF.RIGHT_KA
        )

        self.kinematics = DifferentialDriveKinematics(RobotConst.TRACK_WIDTH)

        self.max_pct_output = self.tunable("Tunables/DrivePCTOUT", 1.0)

        self.forward_slew = SlewRateLimiter(DrivetrainConst.SLEW_FORWARD)
        self.angular_slew = SlewRateLimiter(DrivetrainConst.SLEW_ROTATION)

        self.forward_velocity_controller = AxisController()
        self.angular_velocity_controller = AxisController()

        self._prev_left_setpoint = 0.0
        self._prev_right_setpoint = 0.0

    def get_left_distance(self) -> meters:
        """
        Returns the left encoder distance in total meters traveled.
        """
        left_position = self.front_left_encoder.getPosition()
        return rotations_to_meters(
            motor_rotations=left_position,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_right_distance(self) -> meters:
        """
        Returns the right encoder distance in total meters traveled.
        """
        right_position = self.front_right_encoder.getPosition()
        return rotations_to_meters(
            motor_rotations=right_position,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )
    
    def get_left_voltage(self) -> volts:
        """
        Returns the applied voltage of the left motor in volts.
        """
        return self.front_left_motor.getBusVoltage() * self.front_left_motor.getAppliedOutput()

    def get_right_voltage(self) -> volts:
        """
        Returns the applied voltage of the right motor in volts.
        """
        return self.front_right_motor.getBusVoltage() * self.front_right_motor.getAppliedOutput()
    
    def get_left_velocity(self) -> meters_per_second:
        """
        Returns the current left motor velocity in meters per second
        """
        left_velocity_rpm = self.front_left_encoder.getVelocity()
        left_velocity_rps = rpm_to_rps(left_velocity_rpm)
        return rps_to_mps(
            motor_rps=left_velocity_rps,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )
    
    def get_right_velocity(self) -> meters_per_second:
        """
        Returns the current right motor velocity in meters per second
        """
        right_velocity_rpm = self.front_right_encoder.getVelocity()
        right_velocity_rps = rpm_to_rps(right_velocity_rpm)
        return rps_to_mps(
            motor_rps=right_velocity_rps,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_heading(self) -> Rotation2d:
        """
        Returns the robot CCW heading as a Rotation2d.
        """
        return Rotation2d(degrees_to_radians(self.gyro.getAngle()))

    def get_pose(self) -> Pose2d:
        """
        Returns the robot pose as a Pose2d.
        """
        return self.odometry.getPose()

    def reset_odometry(self, pose: Pose2d = Pose2d()) -> None:
        """
        Resets odometry to the specified pose.
        """
        self.gyro.reset()
        self.front_left_encoder.setPosition(0)
        self.front_right_encoder.setPosition(0)

        self.odometry.resetPosition(
            gyroAngle=self.get_heading(),
            leftDistance=self.get_left_distance(),
            rightDistance=self.get_right_distance(),
            pose=pose
        )

    def reset_heading(self) -> None:
        """
        Resets the robot orientation to zero.
        """
        self.gyro.reset()

    def reset_drivetrain(self) -> None:
        """
        Resets encoders, the gyro heading, odometry, and PIDControllers.
        """
        self.front_left_encoder.setPosition(0)
        self.front_right_encoder.setPosition(0)

        self.reset_heading()
        self.reset_odometry()

        self.left_pid.reset()
        self.right_pid.reset()

        self._prev_left_setpoint = 0.0
        self._prev_right_setpoint = 0.0

    def request_stop(
        self,
        priority: int,
        source: str = "unknown"
    ) -> None:
        """
        Stops all drivetrain movement by setting all requested velocities to zero.

        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        self.forward_velocity_controller.request(
            value=0.0, 
            priority=priority,
            source=source
        )
        self.angular_velocity_controller.request(
            value=0.0, 
            priority=priority,
            source=source
        )
        
        # Reset setpoints to avoid FF voltage jumps.
        self._prev_left_setpoint = 0.0
        self._prev_right_setpoint = 0.0

    def request_forward_percent(
        self,
        forward_speed: percent,
        priority: int,
        source: str = "unknown"
    ) -> None:
        """
        Adds a request for a forward speed percent. Applies slew to the requested forward speed.
        
        :param forward_speed: Desired vertical pct output from [-1, 1].
        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        forward = self.forward_slew.calculate(forward_speed)
        
        forward = clamp(forward, -self.max_pct_output.value, self.max_pct_output.value)

        self.forward_velocity_controller.request(
            value=RobotConst.MAX_SPEED_MPS * forward,
            priority=priority,
            source=source
        )

    def request_angular_percent(
        self,
        angular_speed: percent,
        priority: int,
        source: str = "unknown"
    ) -> None:
        """
        Adds a request for an angular speed percent. Applies slew to the requested angular speed.
        
        :param angular_speed: Desired angular pct output from [-1, 1].
        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        angular_speed = self.angular_slew.calculate(angular_speed)

        angular_speed = clamp(angular_speed, -self.max_pct_output.value, self.max_pct_output.value)
        self.angular_velocity_controller.request(
            value=RobotConst.MAX_SPEED_RADPS * angular_speed,
            priority=priority,
            source=source
        )
    
    def command_left_voltage(self, voltage: volts) -> None:
        """
        Directly commands the left motor given raw voltage. This should only be used for SysID.
        """
        self.front_left_motor.setVoltage(voltage)
    
    def command_right_voltage(self, voltage: volts) -> None:
        """
        Directly commands the right motor given raw voltage. This should only be used for SysID.
        """
        self.front_right_motor.setVoltage(voltage)

    def _tank_drive_velocity(
        self, 
        desired_left_mps: meters_per_second, 
        desired_right_mps: meters_per_second
    ) -> None:
        """
        Drives each side of the drivetrain using closed-loop velocity control.

        Uses feedforward (kS, kV, kA) and PID to compute motor voltage.
        Inputs are wheel linear velocities in meters per second.
        """
        left_velocity_mps = self.get_left_velocity()
        right_velocity_mps = self.get_right_velocity()

        left_ff = self.left_ff.calculate(self._prev_left_setpoint, desired_left_mps)
        right_ff = self.right_ff.calculate(self._prev_right_setpoint, desired_right_mps)

        self._prev_left_setpoint = desired_left_mps
        self._prev_right_setpoint = desired_right_mps

        left_pid = self.left_pid.calculate(left_velocity_mps, desired_left_mps)
        right_pid = self.right_pid.calculate(right_velocity_mps, desired_right_mps)

        self.front_left_motor.setVoltage(
            clamp(left_ff + left_pid, -RobotConst.NOMINAL_VOLTAGE, RobotConst.NOMINAL_VOLTAGE)
        )
        self.front_right_motor.setVoltage(
            clamp(right_ff + right_pid, -RobotConst.NOMINAL_VOLTAGE, RobotConst.NOMINAL_VOLTAGE)
        )

    def publish_telemetry(self) -> None:
        """
        Publishes all drivetrain-specific data before execute() is run.
        """
        self.publish_value("Drive/GyroAngle (radians)", self.get_heading().radians())

        pose = self.get_pose()
        self.publish_value("Drive/Pose/kX (meters)", pose.X())
        self.publish_value("Drive/Pose/kY (meters)", pose.Y())
        self.publish_value("Drive/Pose/kOmega (radians)", pose.rotation().radians())

        self.publish_value("Drive/FLMotorOutput [-1, 1]", self.front_left_motor.getAppliedOutput())
        self.publish_value("Drive/FRMotorOutput [-1, 1]", self.front_right_motor.getAppliedOutput())

        self.publish_value("Drive/FLEncoderPos (meters)",self.get_left_distance())
        self.publish_value("Drive/FREncoderPos (meters)", self.get_right_distance())

        self.publish_value("Drive/FLVelocity (radps)",self.get_left_velocity())
        self.publish_value("Drive/FRVelocity (radps)", self.get_right_velocity())

        self.publish_value("Drive/Chassis/xPose", self.get_pose().X())
        self.publish_value("Drive/Chassis/yPose", self.get_pose().Y())
        self.publish_value("Drive/Chassis/yaw (radians)", self.get_pose().rotation().radians())

        resolved_vx_request = self.forward_velocity_controller.resolve()
        resolved_omega_request = self.angular_velocity_controller.resolve()

        self.publish_value("Drive/DriveAxis/ResolvedVX (m per s)", resolved_vx_request.value)
        self.publish_value("Drive/DriveAxis/ResolvedOmega (rad per s)", resolved_omega_request.value)
        self.publish_value("Drive/DriveAxis/ForwardMode", resolved_vx_request.source)
        self.publish_value("Drive/DriveAxis/RotationMode", resolved_omega_request.source)
        self.publish_value("Drive/DriveAxis/ForwardPriority", resolved_vx_request.priority)
        self.publish_value("Drive/DriveAxis/RotationPriority", resolved_omega_request.priority)

    def execute(self) -> None:
        """
        Updates the robots position on the field and drives by the requested speeds.
        This method is automatically called at the end of each iteration by the scheduler.
        """
        self.odometry.update(
            self.get_heading(),
            self.get_left_distance(),
            self.get_right_distance()
        )

        # Retrieves the requests with the highest priority
        vx = self.forward_velocity_controller.resolve().value
        omega = self.angular_velocity_controller.resolve().value

        chassis = ChassisSpeeds(vx, 0.0, omega)

        wheel_speeds = self.kinematics.toWheelSpeeds(chassis)
        wheel_speeds.desaturate(RobotConst.MAX_SPEED_MPS)

        self._tank_drive_velocity(
            wheel_speeds.left,
            wheel_speeds.right
        )

        self.forward_velocity_controller.clear()
        self.angular_velocity_controller.clear()

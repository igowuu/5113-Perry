from phoenix5 import TalonSRX, ControlMode

from wpimath import applyDeadband
from wpimath.controller import ArmFeedforward
from wpimath.units import percent, radians, radians_per_second

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent
from adaptive_robot.requests import AxisController
from adaptive_robot.hardware.adaptive_dc_encoder import AdaptiveDCEncoder

from config.constants import JoystickConst, RobotConst, ArmConst, ArmVelocityPID, ArmPositionPID, ArmFF
from utils.math_utils import clamp


class Arm(AdaptiveComponent):
    """
    This class declares a single-joined arm, moved by two SRX motors with two absolute encoders.
    """
    def __init__(self, robot: "AdaptiveRobot") -> None:
        super().__init__(robot)

        self.left_arm_motor = TalonSRX(21)
        self.right_arm_motor = TalonSRX(22)

        self.right_arm_motor.setInverted(True)

        self.right_arm_motor.follow(self.left_arm_motor)

        self.left_arm_motor.enableVoltageCompensation(True)
        self.left_arm_motor.configVoltageCompSaturation(RobotConst.NOMINAL_VOLTAGE)

        self.right_arm_motor.enableVoltageCompensation(True)
        self.right_arm_motor.configVoltageCompSaturation(RobotConst.NOMINAL_VOLTAGE)

        self.left_arm_encoder = AdaptiveDCEncoder(0)
        self.right_arm_encoder = AdaptiveDCEncoder(1)

        self.arm_velocity_pid = self.tunablePID(
            kp=ArmVelocityPID.KP, 
            ki=ArmVelocityPID.KI, 
            kd=ArmVelocityPID.KD,
            directory="Tunables/Arm/ArmVelocityPID"
        )
        self.arm_position_pid = self.tunablePID(
            ArmPositionPID.KP, 
            ArmPositionPID.KI, 
            ArmPositionPID.KD,
            directory="Tunables/Arm/ArmPositionPID"
        )

        self.arm_ff = ArmFeedforward(ArmFF.KS, ArmFF.KG, ArmFF.KV, ArmFF.KA)

        self.max_pct_output = self.tunable("Tunables/ArmPCTOUT", 0.8)

        self.arm_velocity_controller = AxisController()

        self._prev_velocity_setpoint = 0.0

    def get_angle(self) -> radians:
        """
        Returns current arm angle in radians (averaged between encoders).
        """
        left_arm_position = self.left_arm_encoder.get_position()
        right_arm_position = self.right_arm_encoder.get_position()

        return (left_arm_position + right_arm_position) / 2.0
    
    def get_velocity(self) -> radians_per_second:
        """
        Returns current arm velocity in radians (averaged between encoders).
        """
        left_arm_velocity = self.left_arm_encoder.get_velocity()
        right_arm_velocity = self.right_arm_encoder.get_velocity()

        return (left_arm_velocity + right_arm_velocity) / 2.0

    def request_stop(
        self,
        priority: int,
        source: str = "unknown"
    ) -> None:
        """
        Stops all arm movement by requesting zero velocity from both motors.

        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        self.arm_velocity_controller.request(
            value=0.0,
            priority=priority,
            source=source
        )

        # Reset velocity setpoint to avoid FF voltage jumps.
        self._prev_velocity_setpoint = 0.0

    def request_move_arm(
        self, 
        desired_pct_output: percent,
        priority: int,
        source: str = "unknown"
    ) -> None:
        """
        Request arm movement with a desired pct output.
        
        :param desired_pct_output: The desired pct output in the range of [-1.0, 1.0].
        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        desired_pct_output = applyDeadband(desired_pct_output, JoystickConst.DEADBAND)

        self.arm_velocity_controller.request(
            value=desired_pct_output * ArmConst.MAX_RADPS, 
            priority=priority,
            source=source
        )
    
    def _move_arm_velocity(self, desired_velocity: radians_per_second) -> None:
        """
        Command both motors given a desired velocity in radps.
        """
        current_angle = self.get_angle()
        current_velocity = self.get_velocity()

        # Set voltage to zero if voltage will push the arm below its safety limit.
        if desired_velocity < 0 and current_angle <= ArmConst.MIN_ANGLE:
            self.left_arm_motor.set(ControlMode.PercentOutput, 0.0)
            return
        
        # Set voltage to zero if voltage will push the arm beyond its safety limit.
        if desired_velocity > 0 and current_angle >= ArmConst.MAX_ANGLE:
            self.left_arm_motor.set(ControlMode.PercentOutput, 0.0)
            return

        ff_volts = self.arm_ff.calculate(current_angle, self._prev_velocity_setpoint, desired_velocity)
        pid_volts = self.arm_velocity_pid.calculate(current_velocity, desired_velocity)

        # Convert from voltage to pct out
        total_pct_out = (ff_volts + pid_volts) / RobotConst.NOMINAL_VOLTAGE
        total_pct_out = clamp(total_pct_out, -self.max_pct_output.value, self.max_pct_output.value)

        self._prev_velocity_setpoint = desired_velocity

        self.left_arm_motor.set(ControlMode.PercentOutput, total_pct_out)

    def move_to_angle(self, target_angle: radians) -> None:
        """
        Moves the arm to a specific angle in radians using position control with PID and feedforward.
        """
        current_angle = self.get_angle()

        target_angle = clamp(target_angle, ArmConst.MIN_ANGLE, ArmConst.MAX_ANGLE)

        desired_velocity = self.arm_position_pid.calculate(current_angle, target_angle)

        self._move_arm_velocity(desired_velocity)

    def publish_telemetry(self) -> None:
        """
        Publishes all arm-specific data before execute() is run.
        """
        arm_angle = self.get_angle()

        self.publish_value("Arm/Angle (radians)", arm_angle)

        self.publish_value("Arm/MinLimitReached", arm_angle <= ArmConst.MIN_ANGLE)
        self.publish_value("Arm/MaxLimitReached", arm_angle >= ArmConst.MAX_ANGLE)

        self.publish_value("Arm/LeftMotorOutput [-1, 1]", self.left_arm_motor.getMotorOutputPercent())
        self.publish_value("Arm/RightMotorOutput [-1, 1]", self.right_arm_motor.getMotorOutputPercent())
        
        resolved_pct_out = self.arm_velocity_controller.resolve()

        self.publish_value("Arm/ArmAxis/Resolved (radps)", resolved_pct_out.value)
        self.publish_value("Arm/ArmAxis/Mode", resolved_pct_out.source)
        self.publish_value("Arm/ArmAxis/Priority", resolved_pct_out.priority)

    def execute(self) -> None:
        """
        Method automatically called at the end of each iteration by the scheduler.
        """
        requested_radps = self.arm_velocity_controller.resolve()

        self._move_arm_velocity(requested_radps.value)

        self.arm_velocity_controller.clear()

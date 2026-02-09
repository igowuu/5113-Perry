from phoenix6.hardware import TalonFX
from phoenix6.configs import MotorOutputConfigs
from phoenix6.signals import NeutralModeValue

from wpimath import applyDeadband
from wpimath.units import percent

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent
from adaptive_robot.requests import AxisController

from utils.math_utils import clamp
from config.constants import JoystickConst


class Intake(AdaptiveComponent):
    """
    This class declares the intake component to grab and release FRC items. The intake component
    is attatched to the arm and is able to grab and release items.
    """
    def __init__(self, robot: "AdaptiveRobot") -> None:
        super().__init__(robot)

        self.intake_motor = TalonFX(23)
        
        self.motor_out_config = MotorOutputConfigs()
        self.motor_out_config.neutral_mode = NeutralModeValue.BRAKE

        self.intake_motor.configurator.apply(self.motor_out_config)

        self.max_pct_output = self.tunable("Tunables/IntakePCTOUT", 0.8)

        self.intake_pct_controller = AxisController()

    def request_stop(self, priority: int, source: str = "unknown") -> None:
        """
        Requests to set the intake motor to zero volts.
        
        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        self.intake_pct_controller.request(
            value=0.0, 
            priority=priority, 
            source=source
        )

    def request_move_intake(
        self, 
        desired_pct_output: percent, 
        priority: int, 
        source: str = "unknown"
    ) -> None:
        """
        Move the intake with a desired pct output.
        
        :param desired_pct_out: Requested pct out in the range of [-1.0, 1.0].
        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        desired_pct_output = applyDeadband(desired_pct_output, JoystickConst.DEADBAND)

        desired_pct_output = clamp(
            desired_pct_output, 
            -self.max_pct_output.value, 
            self.max_pct_output.value
        )

        self.intake_pct_controller.request(
            value=desired_pct_output, 
            priority=priority, 
            source=source
        )

    def execute(self) -> None:
        """
        Method automatically called at the end of each iteration by the scheduler.
        """
        pct_out = self.intake_pct_controller.resolve().value

        self.intake_motor.set(pct_out)

        self.intake_pct_controller.clear()

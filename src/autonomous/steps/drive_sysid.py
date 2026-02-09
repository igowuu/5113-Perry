from autonomous.steps.step_base import Step

from subsystems.drivetrain import Drivetrain

from adaptive_robot.sysid.sysidroutine import Mechanism, Config, SysIdRoutine

class DriveLeftSysId(Step):
    """
    Step to tune left drivetrain FF via adaptive's SysIdRoutine.
    """
    def __init__(self, drivetrain: Drivetrain) -> None:
        self.left_mechanism = Mechanism(
            command_voltage=drivetrain.command_left_voltage,
            get_voltage=drivetrain.get_left_voltage,
            get_position=drivetrain.get_left_distance,
            get_velocity=drivetrain.get_left_velocity,
            name="Left_motors"
        )
        self.left_config = Config()

        self.left_routine = SysIdRoutine(self.left_mechanism, self.left_config, "DRIVETRAIN_LEFT")
        self.left_tests = self.left_routine.generate_all_tests()

        self.current_test_index = 0
        self.current_test = None

    def start(self) -> None:
        self.current_test = self.left_tests[0]
        self.current_test.start()

    def update(self) -> None:
        if not self.current_test:
            return

        self.current_test.step()

        if not self.current_test.is_running():
            self.current_test.stop()

            self.current_test_index += 1

            if self.current_test_index < len(self.left_tests):
                self.current_test = self.left_tests[self.current_test_index]
                self.current_test.start()
            else:
                self.current_test = None

    def is_finished(self) -> bool:
        return self.current_test is None

    def stop(self) -> None:
        if self.current_test:
            self.current_test.stop()

class DriveRightSysId(Step):
    """
    Step to tune right drivetrain FF via adaptive's SysIdRoutine.
    """
    def __init__(self, drivetrain: Drivetrain) -> None:
        self.right_mechanism = Mechanism(
            command_voltage=drivetrain.command_right_voltage,
            get_voltage=drivetrain.get_right_voltage,
            get_position=drivetrain.get_right_distance,
            get_velocity=drivetrain.get_right_velocity,
            name="Right_motors"
        )
        self.right_config = Config()

        self.left_routine = SysIdRoutine(self.right_mechanism, self.right_config, "DRIVETRAIN_RIGHT")
        self.left_tests = self.left_routine.generate_all_tests()

        self.current_test_index = 0
        self.current_test = None

    def start(self) -> None:
        self.current_test = self.left_tests[0]
        self.current_test.start()

    def update(self) -> None:
        if not self.current_test:
            return

        self.current_test.step()

        if not self.current_test.is_running():
            self.current_test.stop()

            self.current_test_index += 1

            if self.current_test_index < len(self.left_tests):
                self.current_test = self.left_tests[self.current_test_index]
                self.current_test.start()
            else:
                self.current_test = None

    def is_finished(self) -> bool:
        return self.current_test is None

    def stop(self) -> None:
        if self.current_test:
            self.current_test.stop()

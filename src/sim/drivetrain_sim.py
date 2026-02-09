from rev import SparkMaxSim
from sim.navx_sim import NavXSim

from wpimath.system.plant import DCMotor
from wpilib.simulation import DifferentialDrivetrainSim

from pyfrc.physics.core import PhysicsInterface

from subsystems.drivetrain import Drivetrain

from utils.math_utils import meters_to_rotations, mps_to_rps, rps_to_rpm
from config.constants import RobotConst, DrivetrainConst


class DrivetrainSim:
    """
    This class simulates the predicted drivetrain movement given input from the Drivetrain subsystem.
    It uses declared constants in the config file to simulate physics.
    """
    def __init__(self, physics_controller: PhysicsInterface, drivetrain: Drivetrain) -> None:
        self.physics_controller = physics_controller
        self.drivetrain = drivetrain
        
        # Initalize all simulated hardware
        self.right_gearbox_sim = SparkMaxSim(self.drivetrain.front_right_motor, DCMotor.NEO(2))
        self.right_gearbox_encoder_sim = self.right_gearbox_sim.getRelativeEncoderSim()

        self.left_gearbox_sim = SparkMaxSim(self.drivetrain.front_left_motor, DCMotor.NEO(2))
        self.left_gearbox_encoder_sim = self.left_gearbox_sim.getRelativeEncoderSim()

        self.diff_drive_sim = DifferentialDrivetrainSim(
            driveMotor=DCMotor.NEO(2),
            gearing=DrivetrainConst.GEAR_RATIO,
            J=DrivetrainConst.MOI,
            mass=RobotConst.MASS,
            wheelRadius=RobotConst.WHEEL_DIAMETER / 2.0,
            trackWidth=RobotConst.TRACK_WIDTH
        )

        self.gyro_sim = NavXSim(self.drivetrain.gyro)

    def execute(self, tm_diff: float) -> None:
        """
        Update all four real SparkMax motor values, all four simulated SparkMax motor values, and gyro,
        allowing us to access the encoder / motor values outside of sim.
        This method is manually called by physics.py.
        """
        battery_voltage = RobotConst.NOMINAL_VOLTAGE

        left_wheel_mps = self.diff_drive_sim.getLeftVelocity()
        right_wheel_mps = self.diff_drive_sim.getRightVelocity()

        left_motor_rps = mps_to_rps(
            velocity=left_wheel_mps,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

        right_motor_rps = mps_to_rps(
            velocity=right_wheel_mps,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

        left_motor_rpm = rps_to_rpm(left_motor_rps)
        right_motor_rpm = rps_to_rpm(right_motor_rps)

        # iterate() is used to update the actual SparkMax objects from the SparkMaxSim velocities.
        self.left_gearbox_sim.iterate(
            velocity=left_motor_rpm,
            vbus=battery_voltage,
            dt=tm_diff
        )

        self.right_gearbox_sim.iterate(
            velocity=right_motor_rpm,
            vbus=battery_voltage,
            dt=tm_diff
        )

        self.diff_drive_sim.setInputs(
            self.left_gearbox_sim.getSetpoint(),
            self.right_gearbox_sim.getSetpoint(),
        )

        self.diff_drive_sim.update(tm_diff)

        left_encoder_pos_m = self.diff_drive_sim.getLeftPosition()
        right_encoder_pos_m = self.diff_drive_sim.getRightPosition()

        self.left_gearbox_encoder_sim.setPosition(
            position=meters_to_rotations(
                distance=left_encoder_pos_m, 
                wheel_diameter=RobotConst.WHEEL_DIAMETER,
                gear_ratio=DrivetrainConst.GEAR_RATIO,
            )
        )
        self.left_gearbox_encoder_sim.setVelocity(
            velocity=rps_to_rpm(left_motor_rps)
        )

        self.right_gearbox_encoder_sim.setPosition(
            position=meters_to_rotations(
                distance=right_encoder_pos_m, 
                wheel_diameter=RobotConst.WHEEL_DIAMETER,
                gear_ratio=DrivetrainConst.GEAR_RATIO
            )
        )
        self.right_gearbox_encoder_sim.setVelocity(
            velocity=rps_to_rpm(right_motor_rps)
        )

        if self.physics_controller.field is not None:
            self.physics_controller.field.setRobotPose(self.diff_drive_sim.getPose())
        
        robot_orientation = self.diff_drive_sim.getHeading().degrees()
        
        self.gyro_sim.update(robot_orientation)

import math

from wpimath.units import (
    kilograms, meters, meters_per_second, meters_per_second_squared, 
    percent, radians, radians_per_second, volts, kilogram_square_meters,
    volt_seconds_per_meter, volt_seconds_squared_per_meter
)

# Percent values are normalized: 1.0 == 100%

class RobotConst:
    MASS: kilograms = 45.3
    WHEEL_DIAMETER: meters = 0.1524
    MAX_SPEED_MPS: meters_per_second = 4.229
    MAX_SPEED_RADPS: radians_per_second = 10.703
    MAX_ACCEL_MPS_SQUARED: meters_per_second_squared = 2.20
    TRACK_WIDTH: meters = 0.6  # Distance between left and right wheels
    NOMINAL_VOLTAGE: volts = 12.0 # Voltage for the battery


# Joystick constants
class JoystickConst:
    DEADBAND: percent = 0.15


# Drivetrain constants
class DrivetrainConst:
    GEAR_RATIO = 10.71  # Motor rotations per wheel rotation
    MOI: kilogram_square_meters = 6.0 # Moment of intertia of drivetrain (estimate)
    SLEW_FORWARD: meters_per_second = 6.5
    SLEW_ROTATION: radians_per_second = 16.3


# Drivetrain PID
class DrivetrainPID:
    # Left gearbox PID (input in m/s, output in volts - tuned)
    LEFT_KP = 0.79921
    LEFT_KI = 0.0
    LEFT_KD = 0.0

    # Right gearbox PID (input in m/s, output in volts - tuned)
    RIGHT_KP = 0.81218
    RIGHT_KI = 0.0
    RIGHT_KD = 0.0

    B = 2.0
    ZETA = 0.7


# Drivetrain feedforward
class DrivetrainFF:
    # Left gearbox feedforward (tuned with SysID)
    LEFT_KS: volts = 0.00064728
    LEFT_KV: volt_seconds_per_meter = 2.7891
    LEFT_KA: volt_seconds_squared_per_meter = 0.46766

    # Right gearbox feedforward (tuned with SysID)
    RIGHT_KS: volts = 0.004742
    RIGHT_KV: volt_seconds_per_meter = 2.8013
    RIGHT_KA: volt_seconds_squared_per_meter = 0.47241


# Arm constants
class ArmConst:
    GEAR_RATIO = 30.0  # motor rotations per arm rotation
    MOI: kilogram_square_meters = 0.1  # Moment of inertia of arm (estimate)
    LENGTH: meters = 0.2  # Arm length

    MIN_ANGLE: radians = 0.0
    MAX_ANGLE: radians = math.pi / 2.0

    APPLIED_PCT_OUT: percent = 1.0
    MAX_RADPS: radians_per_second = 6.3

    # Simulation constants (unitless, all in terms of the GUI)
    SIM_HEIGHT = 50.0
    SIM_WIDTH = 20.0
    ROOT_LENGTH = 10.0
    ROOT_ANGLE = 0.0
    ROOT_X = 10.0
    ROOT_Y = 1.0


# Arm PID
class ArmVelocityPID:
    # Arm velocity PID (input in radps, output in volts - untuned)
    KP = 0.01
    KI = 0.0
    KD = 0.0


# Arm PID
class ArmPositionPID:
    # Arm position PID (input in radps, output in volts - untuned)
    KP = 0.01
    KI = 0.0
    KD = 0.0


class ArmFF:
    # Arm FF (estimates)
    KS: volts = 0.50
    KG: volts = 0.0
    KV: volt_seconds_per_meter = 2.60
    KA: volt_seconds_squared_per_meter = 0.05


# Intake constants
class IntakeConst:
    APPLIED_PCT_OUT: percent = 1.0

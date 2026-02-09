# Perry

**Perry** is the retired 5113 FRC robot software framework, built on top of the [AdaptiveRobot](https://github.com/igowuu/AdaptiveRobot) library. 
It handles drivetrain, arm, and intake subsystems, supporting both autonomous (choreo) and teleoperated modes with a priority-based control system.

## Features

* Priority-based command system for safe and autonomous operation.
* Odometry and kinematics support for drivetrain and arm motion.
* Autonomous routine support with `AutoManager`.
* Real-time telemetry via dashboard.

---

## Installation

1. Clone the repo:

```bash
git clone https://github.com/igowuu/perry.git
cd perry
```

2. Install dependencies (requires Python 3.11+ and `robotpy` libraries):

```bash
pip install -r requirements.txt
```

3. Connect to your FRC robot controller or simulator.

---

## Usage

Run Perry on your robot controller:

```bash
robotpy deploy
```

Control is mapped to:

* Left joystick: drive control
* Right joystick: drive/auxiliary control
* Arm and intake controlled via left joystick buttons

Autonomous routines can be selected in `auto_manager.py` or via code in `autonomous/routines/`.

---

## Project Structure

```
perry/
├─ adaptive_robot/      # Adaptive lib for request-based design
├─ autonomous/          # Autonomous routines and manager
├─ config/              # Subsystem and bot specific constants
├─ controls/            # Joystick mappings for subsystems
├─ deploy/              # Choreo trajectories
├─ sim/                 # Simulated subsystems and hardware
├─ subsystems/          # Subsystems: drivetrain, arm, intake
├─ tests/               # Default pyFRC tests
├─ utils/               # Math, general dashboard publishing
├─ robot.py             # Main robot initialization
├─ physics.py           # Main sim initialization
├─ README.md            # Project overview
├─ requirements.txt     # Dependencies
```

---

## License

MIT License. See [LICENSE](https://opensource.org/license/MIT) for details.

---

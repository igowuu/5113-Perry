# 5113-Perry

![Python](https://img.shields.io/badge/python-3.11-blue)
![RobotPy](https://img.shields.io/badge/RobotPy-2026-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview:

This is a three-subsystem tankbot with a drivetrain, arm, and intake.

The codebase is meant to be for competition-readiness and, most importantly, as a teaching reference for others.

## Structure

robot.py inherits from AdaptiveRobot, a custom architecture that prioritizes safety. More info about it can be found here - I encourage you to take a look: https://github.com/igowuu/AdaptiveRobot.git

The majority of other files - subsystems, independent parts of the robot - inherit from AdaptiveComponent.

AdaptiveRobot and AdaptiveComponent are meant to go hand in hand to schedule the flow of the program, prevent conflicts, enforce safety measures, and increase flexibility of the codebase.

## Files:

`robot.py`
- Controls all components and orchestrates highest-level actions.

`components/drivetrain.py`
- Handles all robot vertical and angular movement, both which can be commanded individually via requests (though Adaptive).

`components/arm.py`
- Controls a single-jointed arm with a built-in intake through requests.

`components/intake.py`
- Controls an intake mechanism that is able to both grab and release objects.

`config/constants.py`
- Holds all subsystem-specific constants, most of which relating to robot measurements.

`controls/`
- Holds all files that request movement for subsystems by reading joystick input.

`autonomous/`
- Holds all autonomous routines. Autonomous routines are broken up into several pieces:
    - manager.py - handles routine selection and overall execution, commanded by robot.py.
    - routines/ - consists of multiple individual steps to complete a larger action.
    - steps/ - small actions per subsystem to complete a small task.

`physics.py`
- Orchestrates and updates all simulation logic

`sim/`
- Holds simulation logic that directly reflects actual logic for each subsystem, accessing values through getters and direct modification of real hardware objects.

And that's it! Obviously this is a massive oversimplification of the structure, but it's a good start for reading the actual codebase.

I highly recommend taking a look at both this codebase and the adaptive structure that it uses, so you understand what is happening fully. I intend this to be both a teaching reference and example for AdaptiveRobot.

Thank you for reading this README and looking at this codebase. If you have the time, I recommend looking at the Adaptive architecture here: https://github.com/igowuu/AdaptiveRobot.git

If you wish to contact me or give me feedback, please reach out! You can talk to me via discord. My username is: `"igowu."`.

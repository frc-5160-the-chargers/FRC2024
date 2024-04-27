# ChargerLib

ChargerLib is 5160's custom kotlin-oriented library, intended for use in 2024 and beyond.


## Features:
- Classes for some common subsystems (e.g. drivetrains)
- Support for units in all values provided through [KMeasure](https://github.com/battery-staple/KMeasure)
- These units also take advantage of kotlin inline classes, which means that during runtime, they are mostly represented as double's: this reduces runtime overhead.
- Units-based wrappers for most WPILib classes, including Pose2d, Pose3d, Trapezoid Profiling, and more.

- Wrapping of components from different manufacturers to allow simulation support and better subsystem compatibility and hardware abstraction , (for example, switching a drivetrain motor from a neo to a falcon and/or a simulated motor becomes trivial.)
- Idiomatic Kotlin Commands DSL for creating commands and autonomous routines
- Custom logging framework that logs to datalog and networktables
- Custom tuning framework that allows for easy and intuitive tuning of components.

## How does it work? Is it mandatory to use or not?

ChargerLib is a "lib folder"-esque library; all imports that come from chargerlib have the frc.chargers import moniker.
The overall purpose of chargerlib is to provide optional components that make robot code more expressive and simpler.

Besides of the use of the ChargerRobot class, everything within ChargerLib is entirely optional to use;
in fact, it is recommended that newer coders use base java features if they have just learned the basics of robot code.

### How Do I learn more?

To learn more, view the other README files listed below. They can be read in any order; 
however, reading them in alphabetical order(A, then B, then C, etc.) is the recommended path.

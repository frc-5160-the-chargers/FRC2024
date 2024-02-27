# ChargerLib Basics: Framework

The framework of ChargerLib is 99% identical to the typical AdvantageKit and Command-based robot frameworks.

Some of these tenets include:
1. A RobotContainer class that acts like the "main" class of the robot; with a minimalistic Robot class that exposes the RobotContainer.
2. Subsystems, which are classes that represent "parts" of the robot(like the arm, climber, wrist, etc.)
3. Commands, which execute actions from subsystems and ensure that a subsystem is not being used in 2 different places,
4. IO layers, which encapsulate low-level functionality(like setting power to motors, fetching voltage and applied current, etc.)
and provides separation between real and simulation components),
5. And logging and replay support.

## Useful information:

[Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html) - 
Illustrates how command-based works. Our command system exactly matches wpilib's;
with the exception of the optional use of buildCommand for DSL-based commands.




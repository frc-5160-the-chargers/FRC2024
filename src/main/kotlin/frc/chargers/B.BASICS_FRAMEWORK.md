# ChargerLib Basics: Framework

The framework of ChargerLib is 99% identical to the typical Command-based robot framework.

Some of these tenets include:
1. A central Robot class, where all button bindings are mapped(and class instances are created), 
2. Subsystems, which are classes that represent "parts" of the robot(like the arm, climber, wrist, etc.)
3. and Commands, which execute actions from subsystems and ensure that a subsystem is not being used in 2 different places.

## Useful information:
[Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html) - 
Illustrates how command-based works. Our command system exactly matches wpilib's;
except the optional use of buildCommand for DSL-based commands.

## So, how do I start off coding a robot?
To start off, create a central Robot class that extends the ChargerRobot abstract class. 
The ChargerRobot class simply extends the typical TimedRobot base class, with useful utilities.

This class is basically the main class of an individual robot; where you would store all your subsystem instances
and map button bindings(in the initializer of the class, or by overriding the robotInit method).

Example:

```kotlin
class CompetitionRobot: ChargerRobot(){
    private val drivetrain = ...
    private val arm = ...
    private val shooter = ...
    
    override fun robotInit(){
        // map button bindings here
    }
    
    init{
        // or in the constructor/initializer here
    }
}
```











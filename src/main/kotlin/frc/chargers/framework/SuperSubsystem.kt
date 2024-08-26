package frc.chargers.framework

import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * A convenience abstract class that defines a subsystem that supports logging and tuning capabilities.
 */
abstract class SuperSubsystem(final override val namespace: String):
    SubsystemBase(), Loggable, Tunable
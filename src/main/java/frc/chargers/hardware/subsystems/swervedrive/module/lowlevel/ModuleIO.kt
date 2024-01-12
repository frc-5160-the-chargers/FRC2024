@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.swervedrive.module.lowlevel

import com.batterystaple.kmeasure.quantities.*

/**
 * An interface representing the low level component of a SwerveModule.
 *
 * There are 2 implementations of this class: one for the real robot ([ModuleIOReal])
 * and one for the sim robot ([ModuleIOSim]).
 */
public interface ModuleIO {
    public val logTab: String

    public val direction: Angle
    public val turnSpeed: AngularVelocity

    public val speed: AngularVelocity
    public val wheelTravel: Angle

    public val driveCurrent: Current
    public val turnCurrent: Current

    public val driveTempCelsius: Double
    public val turnTempCelsius: Double

    // sets the drive and turning voltage by using custom setters
    public var turnVoltage: Voltage
    public var driveVoltage: Voltage
}



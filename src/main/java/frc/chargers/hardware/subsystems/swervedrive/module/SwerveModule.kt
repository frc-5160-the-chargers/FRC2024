@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.swervedrive.module

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.chargers.hardware.subsystems.swervedrive.module.lowlevel.ModuleIO

/**
 * Represents a swerve module within an
 * [frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain].
 *
 * Extends ModuleIO to provide implementing classes
 * with low-level control, I.E setting voltage and getting encoder position.
 */
public interface SwerveModule: ModuleIO {
    public fun setDirection(direction: Angle)

    public fun setDirectionalPower(
        power: Double,
        direction: Angle
    )

    public fun setDirectionalVelocity(
        angularVelocity: AngularVelocity,
        direction: Angle
    )

    public fun getModuleState(wheelRadius: Length): SwerveModuleState

    public fun getModulePosition(wheelRadius: Length): SwerveModulePosition

    public fun halt(){
        turnVoltage = 0.volts
        driveVoltage = 0.volts
    }
}
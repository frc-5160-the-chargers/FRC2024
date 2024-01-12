@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.swervedrive

import com.revrobotics.CANSparkLowLevel
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFXConfiguration
import frc.chargers.hardware.motorcontrol.rev.*
import frc.chargers.hardware.sensors.encoders.Encoder

/**
 * Constructs an instance of [SwerveMotors] with Spark Flex motor controllers.
 */
public inline fun sparkFlexSwerveMotors(
    topLeftId: Int,
    topRightId: Int,
    bottomLeftId: Int,
    bottomRightId: Int,
    configure: ChargerSparkFlexConfiguration.() -> Unit
): SwerveMotors<ChargerSparkFlex> = sparkFlexSwerveMotors(
    ChargerSparkFlex(topLeftId),
    ChargerSparkFlex(topRightId),
    ChargerSparkFlex(bottomLeftId),
    ChargerSparkFlex(bottomRightId),
    configure
)


/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public inline fun sparkMaxSwerveMotors(
    topLeftId: Int,
    topRightId: Int,
    bottomLeftId: Int,
    bottomRightId: Int,
    type: CANSparkLowLevel.MotorType = CANSparkLowLevel.MotorType.kBrushless,
    configure: ChargerSparkMaxConfiguration.() -> Unit = {}
): SwerveMotors<ChargerSparkMax> = sparkMaxSwerveMotors(
        ChargerSparkMax(topLeftId, type),
        ChargerSparkMax(topRightId, type),
        ChargerSparkMax(bottomLeftId, type),
        ChargerSparkMax(bottomRightId, type),
        configure
    )



/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public inline fun talonFXSwerveMotors(
    topLeftId: Int,
    topRightId: Int,
    bottomLeftId: Int,
    bottomRightId: Int,
    configure: ChargerTalonFXConfiguration.() -> Unit = {}
): SwerveMotors<ChargerTalonFX> = talonFXSwerveMotors(
    ChargerTalonFX(topLeftId),
    ChargerTalonFX(topRightId),
    ChargerTalonFX(bottomLeftId),
    ChargerTalonFX(bottomRightId),
    configure
)

/**
 * Constructs an instance of [SwerveMotors] with Spark Max motor controllers.
 */
public inline fun sparkMaxSwerveMotors(
    topLeft: ChargerSparkMax,
    topRight: ChargerSparkMax,
    bottomLeft: ChargerSparkMax,
    bottomRight: ChargerSparkMax,
    configure: ChargerSparkMaxConfiguration.() -> Unit = {}
): SwerveMotors<ChargerSparkMax> {
    val config = ChargerSparkMaxConfiguration().apply(configure)

    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return SwerveMotors(
        topLeft, topRight, bottomLeft, bottomRight,
    )
}


public inline fun sparkFlexSwerveMotors(
    topLeft: ChargerSparkFlex,
    topRight: ChargerSparkFlex,
    bottomLeft: ChargerSparkFlex,
    bottomRight: ChargerSparkFlex,
    configure: ChargerSparkFlexConfiguration.() -> Unit = {}
): SwerveMotors<ChargerSparkFlex>{
    val config = ChargerSparkFlexConfiguration().apply(configure)

    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return SwerveMotors(
        topLeft, topRight, bottomLeft, bottomRight,
    )
}



/**
 * Constructs an instance of [SwerveMotors] with TalonFX motor controllers.
 */
public inline fun talonFXSwerveMotors(
    topLeft: ChargerTalonFX,
    topRight: ChargerTalonFX,
    bottomLeft: ChargerTalonFX,
    bottomRight: ChargerTalonFX,
    configure: ChargerTalonFXConfiguration.() -> Unit = {}
): SwerveMotors<ChargerTalonFX>{
    val config = ChargerTalonFXConfiguration().apply(configure)

    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return SwerveMotors(
        topLeft, topRight, bottomLeft, bottomRight,
    )
}

/**
 * A Helper class to store a group of motors needed for an [EncoderHolonomicDrivetrain];
 * these can be either for turning or driving.
 */
public data class SwerveMotors<out M: EncoderMotorController>(
    val topLeft: M,
    val topRight: M,
    val bottomLeft: M,
    val bottomRight: M
){
    // inline reified allows for runtime checks of whether or not the motors
    // are a specified type.
    public inline fun <reified T: EncoderMotorController> containsMotors(): Boolean =
        topLeft is T && topRight is T && bottomLeft is T && bottomRight is T


    public fun getEncoders(): SwerveEncoders<Encoder> =
        SwerveEncoders(
            topLeft.encoder,
            topRight.encoder,
            bottomLeft.encoder,
            bottomRight.encoder
        )

    public fun forEach(predicate: (M) -> Unit){
        predicate(topLeft)
        predicate(topRight)
        predicate(bottomLeft)
        predicate(bottomRight)
    }
}
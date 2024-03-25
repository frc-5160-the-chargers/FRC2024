@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.Angle
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.encoders.ResettableEncoder
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoderConfiguration
import frc.chargers.hardware.sensors.withOffset

/**
 * Constructs an instance of [SwerveEncoders] with CTRE CANcoders.
 */
public inline fun swerveCANcoders(
    topLeftId: Int,
    topRightId: Int,
    bottomLeftId: Int,
    bottomRightId: Int,
    useAbsoluteSensor: Boolean,
    configure: ChargerCANcoderConfiguration.() -> Unit = {}
): SwerveEncoders<ResettableEncoder> = swerveCANcoders(
    ChargerCANcoder(topLeftId),
    ChargerCANcoder(topRightId),
    ChargerCANcoder(bottomLeftId),
    ChargerCANcoder(bottomRightId),
    useAbsoluteSensor, configure
)

/**
 * Constructs an instance of [SwerveEncoders] with CTRE CANcoders.
 */
public inline fun swerveCANcoders(
    topLeft: ChargerCANcoder,
    topRight: ChargerCANcoder,
    bottomLeft: ChargerCANcoder,
    bottomRight: ChargerCANcoder,
    useAbsoluteSensor: Boolean,
    configure: ChargerCANcoderConfiguration.() -> Unit = {}
): SwerveEncoders<ResettableEncoder> {
    val config = ChargerCANcoderConfiguration().apply(configure)
    topLeft.configure(config)
    topRight.configure(config)
    bottomLeft.configure(config)
    bottomRight.configure(config)

    return if (useAbsoluteSensor){
        SwerveEncoders(
            topLeft.absolute,
            topRight.absolute,
            bottomLeft.absolute,
            bottomRight.absolute
        )
    }else{
        SwerveEncoders(
            topLeft,
            topRight,
            bottomLeft,
            bottomRight
        )
    }
}

/**
 * A utility class that holds absolute encoders for a holonomic drivetrain.
 */
public data class SwerveEncoders <out E: PositionEncoder> (
    val topLeft: E,
    val topRight: E,
    val bottomLeft: E,
    val bottomRight: E
){
    public fun toList(): List<E> =
        listOf(topLeft, topRight, bottomLeft, bottomRight)

    /**
     * Creates an instance of a [SwerveEncoders] object
     *
     * with the specified offsets.
     */
    public fun withOffsets(
        topLeftZero: Angle,
        topRightZero: Angle,
        bottomLeftZero: Angle,
        bottomRightZero: Angle
    ): SwerveEncoders<PositionEncoder> = SwerveEncoders(
        topLeft = topLeft.withOffset(topLeftZero),
        topRight = topRight.withOffset(topRightZero),
        bottomLeft = bottomLeft.withOffset(bottomLeftZero),
        bottomRight = bottomRight.withOffset(bottomRightZero),
    )

    /**
     * A utility function that performs a predicate on each motor stored within the class.
     */
    public fun forEach(predicate: (E) -> Unit){
        predicate(topLeft)
        predicate(topRight)
        predicate(bottomLeft)
        predicate(bottomRight)
    }

    /**
     * A utility function that checks if the encoders specified are a specific type.
     *
     * Takes advantage of reified generics to accomplish this.
     */
    public inline fun <reified T: PositionEncoder> containsEncoders(): Boolean =
        topLeft is T && topRight is T && bottomLeft is T && bottomRight is T

}

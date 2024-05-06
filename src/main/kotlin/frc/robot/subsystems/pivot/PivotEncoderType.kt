package frc.robot.subsystems.pivot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import frc.chargers.hardware.sensors.encoders.PositionEncoder

sealed class PivotEncoderType {
    /**
     * Represents an encoder type where the absolute encoder is "integrated"/plugged into the motor.
     * in other words, the motor is configured to return absolute encoder readings
     * from motor.encoder.angularPosition.
     */
    data class IntegratedAbsoluteEncoder(
        val offset: Angle = 0.degrees
    ): PivotEncoderType()

    /**
     * Represents an encoder type where there is no absolute encoder,
     * and a relative encoder is used for positions.
     */
    data class IntegratedRelativeEncoder(
        val motorGearRatio: Double,
        val startingAngle: Angle = PivotAngle.STARTING // represents the starting position of the pivot; no offset used because of relative encoders
    ): PivotEncoderType()

    /**
     * Represents an encoder type where an external absolute encoder is used.
     */
    data class ExternalAbsoluteEncoder(
        val absoluteEncoder: PositionEncoder,
        val motorGearRatio: Double,
        val offset: Angle = 0.degrees
    ): PivotEncoderType()
}
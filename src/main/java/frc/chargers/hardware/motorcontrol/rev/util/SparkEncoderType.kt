package frc.chargers.hardware.motorcontrol.rev.util

import com.batterystaple.kmeasure.quantities.Time

sealed class SparkEncoderType{
    /**
     * Represents a regular Spark Flex encoder.
     */
    data class Regular(
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkEncoderType()

    /**
     * Represents a Quadrature encoder hooked up to a spark max.
     * On a Spark Max, this is an alternate encoder; on the spark flex, it is an external encoder.
     */
    data class Quadrature(
        val countsPerRev: Int,
        val encoderMeasurementPeriod: Time? = null,
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkEncoderType()

    /**
     * Represents an absolute encoder connected to a Spark Flex.
     */
    data class DutyCycle(
        val averageDepth: Int? = null,
        val inverted: Boolean? = null
    ): SparkEncoderType()
}
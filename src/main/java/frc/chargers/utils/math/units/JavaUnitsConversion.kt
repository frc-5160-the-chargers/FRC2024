@file:Suppress("unused")
package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage as WPIVoltageUnit
import edu.wpi.first.units.Time as WPITimeUnit
import edu.wpi.first.units.Angle as WPIAngleUnit
import edu.wpi.first.units.Distance as WPIDistanceUnit
import edu.wpi.first.units.Current as WPICurrentUnit
import edu.wpi.first.units.Velocity as WPIRate

/*
Extension functions that convert WPILib Measures to kmeasure Quantities and vise versa.
 */


@JvmName("toKmeasureVoltage")
fun Measure<WPIVoltageUnit>.toKmeasure(): Voltage = Voltage(baseUnitMagnitude())

@JvmName("toWPIVoltage")
fun Voltage.toWPI(): Measure<WPIVoltageUnit> = Units.Volts.of(siValue)



@JvmName("toKmeasureTime")
fun Measure<WPITimeUnit>.toKmeasure(): Time = Time(baseUnitMagnitude())

@JvmName("toWPITime")
fun Time.toWPI(): Measure<WPITimeUnit> = Units.Seconds.of(siValue)



@JvmName("toKmeasureDistance")
fun Measure<WPIDistanceUnit>.toKmeasure(): Distance = Distance(baseUnitMagnitude())

@JvmName("toWPIDistance")
fun Distance.toWPI(): Measure<WPIDistanceUnit> = Units.Meters.of(siValue)



@JvmName("toKmeasureAngle")
fun Measure<WPIAngleUnit>.toKmeasure(): Angle = Angle(baseUnitMagnitude())

@JvmName("toWPIAngle")
fun Angle.toWPI(): Measure<WPIAngleUnit> = Units.Radians.of(siValue)



@JvmName("toKmeasureCurrent")
fun Measure<WPICurrentUnit>.toKmeasure(): Current = Current(baseUnitMagnitude())

@JvmName("toWPICurrent")
fun Current.toWPI(): Measure<WPICurrentUnit> = Units.Amps.of(siValue)



private typealias WPIVelocityUnit = WPIRate<WPIDistanceUnit>
private typealias WPIAccelerationUnit = WPIRate<WPIVelocityUnit>
private typealias WPIAngularVelocityUnit = WPIRate<WPIAngleUnit>
private typealias WPIAngularAccelerationUnit = WPIRate<WPIAngularVelocityUnit>



@JvmName("toKmeasureVelocity")
fun Measure<WPIVelocityUnit>.toKmeasure(): Velocity = Velocity(baseUnitMagnitude())

@JvmName("toWPIVelocity")
fun Velocity.toWPI(): Measure<WPIVelocityUnit> = Units.MetersPerSecond.of(siValue)



@JvmName("toKmeasureAcceleration")
fun Measure<WPIAccelerationUnit>.toKmeasure(): Acceleration = Acceleration(baseUnitMagnitude())

@JvmName("toWPIAcceleration")
fun Acceleration.toWPI(): Measure<WPIAccelerationUnit> = Units.MetersPerSecondPerSecond.of(siValue)



@JvmName("toKmeasureAngularVelocity")
fun Measure<WPIAngularVelocityUnit>.toKmeasure(): AngularVelocity = AngularVelocity(baseUnitMagnitude())

@JvmName("toWPIAngularVelocity")
fun AngularVelocity.toWPI(): Measure<WPIAngularVelocityUnit> = Units.RadiansPerSecond.of(siValue)



@JvmName("toKmeasureAngularAcceleration")
fun Measure<WPIAngularAccelerationUnit>.toKmeasure(): AngularAcceleration = AngularAcceleration(baseUnitMagnitude())

@JvmName("toWPIAngularAcceleration")
fun AngularAcceleration.toWPI(): Measure<WPIAngularAccelerationUnit> = Units.RadiansPerSecond.per(Units.Second).of(siValue)






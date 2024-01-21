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



fun Measure<WPIVoltageUnit>.toKmeasure(): Voltage = Voltage(baseUnitMagnitude())

fun Voltage.toWPI(): Measure<WPIVoltageUnit> = Units.Volts.of(siValue)



fun Measure<WPITimeUnit>.toKmeasure(): Time = Time(baseUnitMagnitude())

fun Time.toWPI(): Measure<WPITimeUnit> = Units.Seconds.of(siValue)



fun Measure<WPIDistanceUnit>.toKmeasure(): Distance = Distance(baseUnitMagnitude())

fun Distance.toWPI(): Measure<WPIDistanceUnit> = Units.Meters.of(siValue)



fun Measure<WPIAngleUnit>.toKmeasure(): Angle = Angle(baseUnitMagnitude())

fun Angle.toWPI(): Measure<WPIAngleUnit> = Units.Radians.of(siValue)



fun Measure<WPICurrentUnit>.toKmeasure(): Current = Current(baseUnitMagnitude())

fun Current.toWPI(): Measure<WPICurrentUnit> = Units.Amps.of(siValue)



private typealias WPIVelocityUnit = WPIRate<WPIDistanceUnit>
private typealias WPIAccelerationUnit = WPIRate<WPIVelocityUnit>
private typealias WPIAngularVelocityUnit = WPIRate<WPIAngleUnit>
private typealias WPIAngularAccelerationUnit = WPIRate<WPIAngularVelocityUnit>



fun Measure<WPIVelocityUnit>.toKmeasure(): Velocity = Velocity(baseUnitMagnitude())

fun Velocity.toWPI(): Measure<WPIVelocityUnit> = Units.MetersPerSecond.of(siValue)



fun Measure<WPIAccelerationUnit>.toKmeasure(): Acceleration = Acceleration(baseUnitMagnitude())

fun Acceleration.toWPI(): Measure<WPIAccelerationUnit> = Units.MetersPerSecondPerSecond.of(siValue)



fun Measure<WPIAngularVelocityUnit>.toKmeasure(): AngularVelocity = AngularVelocity(baseUnitMagnitude())

fun AngularVelocity.toWPI(): Measure<WPIAngularVelocityUnit> = Units.RadiansPerSecond.of(siValue)



fun Measure<WPIAngularAccelerationUnit>.toKmeasure(): AngularAcceleration = AngularAcceleration(baseUnitMagnitude())

fun AngularAcceleration.toWPI(): Measure<WPIAngularAccelerationUnit> = Units.RadiansPerSecond.per(Units.Second).of(siValue)






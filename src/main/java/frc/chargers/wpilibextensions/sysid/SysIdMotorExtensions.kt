@file:Suppress("unused")
package frc.chargers.wpilibextensions.sysid

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import frc.chargers.utils.math.units.toWPI

fun SysIdRoutineLog.MotorLog.voltage(voltage: Voltage): SysIdRoutineLog.MotorLog =
    this.voltage(voltage.toWPI())



fun SysIdRoutineLog.MotorLog.angularPosition(position: Angle): SysIdRoutineLog.MotorLog =
    this.angularPosition(position.toWPI())

fun SysIdRoutineLog.MotorLog.linearPosition(position: Distance): SysIdRoutineLog.MotorLog =
    this.linearPosition(position.toWPI())




fun SysIdRoutineLog.MotorLog.angularVelocity(velocity: AngularVelocity): SysIdRoutineLog.MotorLog =
    this.angularVelocity(velocity.toWPI())

fun SysIdRoutineLog.MotorLog.linearVelocity(velocity: Velocity): SysIdRoutineLog.MotorLog =
    this.linearVelocity(velocity.toWPI())



fun SysIdRoutineLog.MotorLog.angularAcceleration(acceleration: AngularAcceleration): SysIdRoutineLog.MotorLog =
    this.angularAcceleration(acceleration.toWPI())

fun SysIdRoutineLog.MotorLog.linearAcceleration(acceleration: Acceleration): SysIdRoutineLog.MotorLog =
    this.linearAcceleration(acceleration.toWPI())




fun SysIdRoutineLog.MotorLog.current(current: Current): SysIdRoutineLog.MotorLog =
    this.current(current.toWPI())




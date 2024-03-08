@file:Suppress("unused")
package frc.robot.hardware.subsystems.climber

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Voltage
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.utils.Precision
import frc.chargers.utils.math.equations.epsilonEquals
import frc.chargers.utils.within
import frc.robot.hardware.subsystems.climber.lowlevel.ClimberIO





// change this to voltage requests / positions requests
// and/or make it stall in the same position when no power is put
class Climber(
    private val io: ClimberIO,

    private val climbMaxVoltage: Voltage = 8.volts,

    val highLimit: Angle? = null,
    val lowLimit: Angle? = null,

    climbPIDConstants: PIDConstants = PIDConstants(0,0,0),
    private val climbSetpointPrecision: Precision<AngleDimension> = Precision.Within(2.degrees)
): SubsystemBase() {
    private val leftController = SuperPIDController(
        climbPIDConstants,
        getInput = { io.leftPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts,
    )

    private val rightController = SuperPIDController(
        climbPIDConstants,
        getInput = { io.rightPosition },
        target = 0.degrees,
        outputRange = (-12).volts..12.volts,
    )

    private var leftHookHoldPosition: Angle? = null
    private var rightHookHoldPosition: Angle? = null

    private fun surpassedLimit(hookSpeed: Double, position: Angle): Boolean{
        val surpassedUpperLimit: Boolean = highLimit != null && hookSpeed > 0.0 && position >= highLimit
        val surpassedLowerLimit: Boolean = highLimit != null && hookSpeed < 0.0 && position >= highLimit

        return surpassedUpperLimit || surpassedLowerLimit
    }

    fun moveLeftHook(speed: Double){
        if (speed epsilonEquals 0.0){
            if (leftHookHoldPosition == null){
                leftHookHoldPosition = io.leftPosition
            }
            moveLeftHook(leftHookHoldPosition!!)
        }else if (surpassedLimit(speed, io.leftPosition)){
            io.setLeftVoltage(0.volts)
            leftHookHoldPosition = null
        }else{
            io.setLeftVoltage(speed * climbMaxVoltage)
            leftHookHoldPosition = null
        }
    }

    fun moveRightHook(speed: Double){
        if (speed epsilonEquals 0.0){
            if (rightHookHoldPosition == null) {
                rightHookHoldPosition = io.rightPosition
            }
            moveRightHook(rightHookHoldPosition!!)
        }else if (surpassedLimit(speed, io.rightPosition)){
            io.setRightVoltage(0.volts)
            rightHookHoldPosition = null
        }else{
            io.setRightVoltage(speed * climbMaxVoltage)
            rightHookHoldPosition = null
        }
    }

    fun moveLeftHook(angularPosition: Angle, voltageRange: ClosedRange<Voltage>? = null){
        var output = leftController.calculateOutput(angularPosition)

        if (voltageRange != null) output = output.coerceIn(voltageRange)

        if (leftController.error.within(climbSetpointPrecision)) output = 0.volts

        io.setLeftVoltage(output)
    }

    fun moveRightHook(angularPosition: Angle, voltageRange: ClosedRange<Voltage>? = null){
        var output = rightController.calculateOutput(angularPosition)

        if (voltageRange != null) output = output.coerceIn(voltageRange)

        if (rightController.error.within(climbSetpointPrecision)) output = 0.volts

        io.setRightVoltage(output)
    }
}
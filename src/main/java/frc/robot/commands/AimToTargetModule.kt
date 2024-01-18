package frc.robot.commands

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.units.degrees
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.Alert

val DEFAULT_AIMING_PID = PIDConstants(0.2,0.0,0.0)
val DEFAULT_AIMING_PRECISION = Precision.Within(Scalar(0.02))
private val noTargetFoundAlert = Alert.warning(text = "A command is attempting to aim to an apriltag, but none can be found.")




/**
 * A helper class for aiming to a specified vision target.
 */
open class AimToTargetModule <T: VisionTarget> (
    val vision: VisionPipeline<T>,
    aimingPID: PIDConstants = DEFAULT_AIMING_PID,
    private val aimPrecision: Precision.Within<ScalarDimension> = DEFAULT_AIMING_PRECISION
){

    private val controller = SuperPIDController(
        aimingPID,
        { Scalar(vision.bestTarget?.tx ?: 0.0) },
        target = Scalar(0.0),
        outputRange = Scalar(-0.5)..Scalar(0.5),
        selfSustain = true
    )

    fun getStrafe(): Double = if (
        canFindTarget() &&
        controller.error !in aimPrecision.allowableError
    ){
        controller.calculateOutput().siValue
    }else{
        0.0
    }

    fun canFindTarget(): Boolean {
        val bestTarget = vision.bestTarget

        return if (bestTarget == null){
            noTargetFoundAlert.active = true
            false
        } else {
            true
        }
    }
}
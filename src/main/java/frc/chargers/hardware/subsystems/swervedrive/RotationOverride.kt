@file:Suppress("unused")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.motionprofiling.MotionProfile
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.sensors.vision.ObjectVisionPipeline
import frc.chargers.utils.Precision
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit
import kotlin.math.PI

/**
 * Represents a supplier of a rotation override;
 * which has access to the drivetrain object reference,
 * and returns a nullable [RotationOverrideResult].
 */
typealias RotationOverride = (EncoderHolonomicDrivetrain) -> RotationOverrideResult?


/**
 * A class that represents a rotation override result for an [EncoderHolonomicDrivetrain].
 */
data class RotationOverrideResult(
    val openLoopRotation: Double,
    val closedLoopRotation: AngularVelocity
)



class AimToAngleRotationOverride(
    private val targetAngle: Angle,
    pidConstants: PIDConstants,
    private val motionProfile: AngularMotionProfile? = null,
    private val aimPrecision: Precision<AngleDimension> = Precision.AllowOvershoot
): RotationOverride {

    private lateinit var motionState: AngularMotionProfileState

    private val pidController = PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD).apply{
        enableContinuousInput(0.0, 2 * PI)

        if (aimPrecision is Precision.Within){
            this.setTolerance( (aimPrecision.allowableError.endInclusive - aimPrecision.allowableError.start).siValue )
        }
    }

    private var previousT = fpgaTimestamp()

    private fun fetchDT(): Time{
        val currentT = fpgaTimestamp()
        val dt = currentT - previousT
        previousT = currentT
        return dt
    }

    override fun invoke(drivetrain: EncoderHolonomicDrivetrain): RotationOverrideResult {
        val output: Double

        if (motionProfile != null){
            if (::motionState.isInitialized){
                motionState = motionProfile.calculate(
                    fetchDT(),
                    setpoint = motionState,
                    goal = AngularMotionProfileState(targetAngle)
                )

                output = pidController.calculate(
                    drivetrain.heading.siValue,
                    motionState.position.siValue
                )
            }else{
                // initialize motion profile state if it is uninitialized
                motionState = AngularMotionProfileState(drivetrain.heading)
                output = 0.0
            }
        }else{
            output = pidController.calculate(
                drivetrain.heading.siValue.inputModulus(0.0, 2 * PI),
                targetAngle.siValue.inputModulus(0.0, 2 * PI)
            )
        }

        return RotationOverrideResult(
            output,
            output * drivetrain.maxRotationalVelocity
        )
    }
}



class AimToObjectRotationOverride  (
    private val visionSystem: ObjectVisionPipeline,
    openLoopRotationPID: PIDConstants,
    private val aimPrecision: Precision<ScalarDimension> = Precision.AllowOvershoot,
    // we don't care what kind of motion profile
    private val motionProfile: MotionProfile<*, *>? = null
): RotationOverride { // implements function type in order to be passed into rotation override acceptor
    private val pidController = PIDController(openLoopRotationPID.kP, openLoopRotationPID.kI, openLoopRotationPID.kD).apply{
        if (aimPrecision is Precision.Within){
            this.setTolerance( (aimPrecision.allowableError.endInclusive - aimPrecision.allowableError.start).siValue )
        }
    }

    val atSetpoint: Boolean get() = pidController.atSetpoint()

    override fun invoke(drivetrain: EncoderHolonomicDrivetrain): RotationOverrideResult? {
        // if no targets are found, don't override rotation
        val bestTarget = visionSystem.bestTarget ?: return null
        val output = pidController.calculate(bestTarget.tx, 0.0)
        return RotationOverrideResult(
            output,
            output * drivetrain.maxRotationalVelocity
        )
    }
}


class AimToAprilTagRotationOverride(
    private val fiducialId: Int,
    private val visionSystem: AprilTagVisionPipeline,
    visionAimPID: PIDConstants,
    visionAimPrecision: Precision<ScalarDimension> = Precision.AllowOvershoot,
    /**
     * Option to control whether the rotation override
     * should aim to the AprilTag's pose instead of a detected target
     * when no targets are found.
     */
    private val aimToTagPoseIfNotFound: Boolean = false,
    poseAimPID: PIDConstants = visionAimPID,
    poseAimPrecision: Precision<AngleDimension> = Precision.AllowOvershoot
): RotationOverride {
    private val visionAimController = PIDController(visionAimPID.kP, visionAimPID.kI, visionAimPID.kD).apply{
        if (visionAimPrecision is Precision.Within){
            this.setTolerance( (visionAimPrecision.allowableError.endInclusive - visionAimPrecision.allowableError.start).siValue )
        }
    }

    private val poseAimController = PIDController(poseAimPID.kP, poseAimPID.kI, poseAimPID.kD).apply{
        if (poseAimPrecision is Precision.Within){
            this.setTolerance( (poseAimPrecision.allowableError.endInclusive - poseAimPrecision.allowableError.start).siValue )
        }
    }



    val atSetpoint: Boolean get() = visionAimController.atSetpoint()


    private val apriltagPose =
        ChargerRobot.APRILTAG_LAYOUT
            .getTagPose(fiducialId)
            .orElseThrow()
            .toPose2d()
            .ofUnit(meters)


    override fun invoke(drivetrain: EncoderHolonomicDrivetrain): RotationOverrideResult? {
        // if no targets are found, don't override rotation
        val allTargets = visionSystem.visionTargets

        for (visionTarget in allTargets){
            if (visionTarget.fiducialId != fiducialId){
                continue
            }

            val output = visionAimController.calculate(visionTarget.tx, 0.0)

            // refreshes the pose aim controller
            poseAimController.calculate(0.0)

            return RotationOverrideResult(
                output,
                output * drivetrain.maxRotationalVelocity
            )
        }

        visionAimController.calculate(0.0)

        if (aimToTagPoseIfNotFound){
            val drivetrainToTagTransform = (apriltagPose - drivetrain.poseEstimator.robotPose)
            // calculates the target heading needed to aim to the apriltag pose
            val targetHeading = atan2(drivetrainToTagTransform.y.inUnit(meters), drivetrainToTagTransform.x.inUnit(meters))
            // uses pid control to reach that pose
            val output = poseAimController.calculate(
                drivetrain.heading.siValue, targetHeading.siValue
            )

            return RotationOverrideResult(
                output,
                output * drivetrain.maxRotationalVelocity
            )
        }else{
            poseAimController.calculate(0.0)
            return null
        }
    }

}




@file:Suppress("unused")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.atan2
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.motionprofiling.AngularMotionProfile
import frc.chargers.controls.motionprofiling.AngularMotionProfileState
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.utils.Precision
import frc.chargers.utils.math.inputModulus
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.external.limelight.LimelightHelpers
import org.photonvision.PhotonCamera
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

/**
 * Provides a backup for a rotation override in case it doesn't detect a target.
 */
fun RotationOverride.withBackup(backup: RotationOverride): RotationOverride = { drivetrain ->
    val initialResult = this@withBackup.invoke(drivetrain)
    val backupResult = backup.invoke(drivetrain)
    initialResult ?: backupResult
}


open class AimToAngleRotationOverride(
    private val getTarget: (EncoderHolonomicDrivetrain) -> Angle,
    angleToVelocityPID: PIDConstants,
    private val motionProfile: AngularMotionProfile? = null,
    private val aimPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    private val invert: Boolean = false
): RotationOverride {

    private lateinit var motionState: AngularMotionProfileState

    private val pidController = PIDController(angleToVelocityPID.kP, angleToVelocityPID.kI, angleToVelocityPID.kD).apply{
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
        val targetAngle = getTarget(drivetrain)
        var output: Double

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

        output *= if (invert) -1.0 else 1.0

        return RotationOverrideResult(
            output / drivetrain.maxRotationalVelocity.siValue,
            AngularVelocity(output),
        )
    }
}



class AimToObjectRotationOverride (
    private val getCrosshairOffset: () -> Double?,
    cameraYawToVelocityPID: PIDConstants,
    private val aimPrecision: Precision<ScalarDimension> = Precision.AllowOvershoot,
    private val invert: Boolean = false
): RotationOverride { // implements function type in order to be passed into rotation override acceptor

    companion object{
        fun fromLimelight(
            camName: String,
            cameraYawToVelocityPID: PIDConstants,
            aimPrecision: Precision<ScalarDimension> = Precision.AllowOvershoot,
            invert: Boolean = false
        ): AimToObjectRotationOverride =
            AimToObjectRotationOverride(
                { if (LimelightHelpers.getTV(camName)) LimelightHelpers.getTX(camName) else null },
                cameraYawToVelocityPID, aimPrecision, invert
            )

        fun fromPhotonCamera(
            photonCamera: PhotonCamera,
            cameraYawToVelocityPID: PIDConstants,
            aimPrecision: Precision<ScalarDimension> = Precision.AllowOvershoot,
            invert: Boolean = false
        ): AimToObjectRotationOverride =
            AimToObjectRotationOverride(
                { photonCamera.latestResult.bestTarget?.yaw },
                cameraYawToVelocityPID, aimPrecision, invert
            )
    }

    private val pidController = PIDController(cameraYawToVelocityPID.kP, cameraYawToVelocityPID.kI, cameraYawToVelocityPID.kD).apply{
        if (aimPrecision is Precision.Within){
            this.setTolerance( (aimPrecision.allowableError.endInclusive - aimPrecision.allowableError.start).siValue )
        }
    }

    private var previousTY: Double? = null

    val atSetpoint: Boolean get() = pidController.atSetpoint()

    override fun invoke(drivetrain: EncoderHolonomicDrivetrain): RotationOverrideResult? {
        // if no targets are found, don't override rotation
        val txValue = getCrosshairOffset() ?: return null
        val output = pidController.calculate(txValue, 0.0) * if (invert) -1.0 else 1.0
        return RotationOverrideResult(
            output / drivetrain.maxRotationalVelocity.siValue,
            AngularVelocity(output)
        )
    }
}


class AimToPoseRotationOverride(
    private val getTarget: () -> UnitPose2d,
    private val angleOffset: Angle = Angle(0.0),
    angleToVelocityPID: PIDConstants,
    motionProfile: AngularMotionProfile? = null,
    aimPrecision: Precision<AngleDimension> = Precision.AllowOvershoot,
    invert: Boolean = false
): AimToAngleRotationOverride(
    { drivetrain ->
        val drivetrainToTagTranslation = (getTarget().translation - drivetrain.robotPose.translation)
        // custom atan2 overload that takes 2 Distances and returns an Angle.
        atan2(drivetrainToTagTranslation.y, drivetrainToTagTranslation.x) + angleOffset
    },
    angleToVelocityPID,
    motionProfile,
    aimPrecision,
    invert
)




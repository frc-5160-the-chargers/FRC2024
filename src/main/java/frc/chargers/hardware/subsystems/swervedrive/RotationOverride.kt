@file:Suppress("unused")
package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.AngularVelocity
import com.batterystaple.kmeasure.quantities.Scalar
import edu.wpi.first.math.controller.PIDController
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.imu.gyroscopes.HeadingProvider
import frc.chargers.hardware.sensors.vision.VisionPipeline
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.utils.Precision
import frc.chargers.utils.math.inputModulus
import frc.chargers.utils.within
import kotlin.math.PI

/**
 * A utility class that stores useful rotation overrides for an [EncoderHolonomicDrivetrain].
 */
object RotationOverride {

    /**
     * A rotation override that aims to a specific angle continuously,
     * which only applies to open loop control.
     *
     * This class does not have a closed loop equivalent.
     */
    class AimToAngleOpenLoop(
        /**
         * This represents a generic heading source; including a drivetrain and/or a NavX.
         */
        private val headingProvider: HeadingProvider,
        private val targetAngle: Angle,
        pidConstants: PIDConstants,
        private val aimPrecision: Precision<ScalarDimension> = Precision.Within(Scalar(0.02))
    ): () -> Double? {
        private val pidController = PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD).apply{
            enableContinuousInput(0.0, 2 * PI)
        }

        override fun invoke(): Double {
            val heading = headingProvider.heading
            val output = pidController.calculate(
                heading.siValue.inputModulus(0.0, 2 * PI),
                targetAngle.siValue.inputModulus(0.0, 2 * PI)
            )

            return if ((heading - targetAngle).within(aimPrecision)) 0.0 else output
        }
    }

    /**
     * A rotation override that aims to the nearest target specified by the vision system supplied,
     * which only applies to open loop control.
     */
    class AimToTargetOpenLoop <T: VisionTarget> (
        private val visionSystem: VisionPipeline<T>,
        pidConstants: PIDConstants,
        private val aimPrecision: Precision<ScalarDimension> = Precision.Within(Scalar(0.02))
    ): () -> Double? { // implements function type in order to be passed into rotation override acceptor
        private val pidController = PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD)

        override fun invoke(): Double? {
            // if no targets are found, don't override rotation
            val bestTarget = visionSystem.bestTarget ?: return null

            // chargerlib extension function
            if (Scalar(bestTarget.tx).within(aimPrecision)){
                return 0.0
            }

            return pidController.calculate(bestTarget.tx, 0.0)
        }
    }


    /**
     * A rotation override that aims to the nearest target specified by the vision system supplied,
     * which only applies to closed loop control.
     */
    class AimToTargetClosedLoop <T: VisionTarget> (
        private val visionSystem: VisionPipeline<T>,
        pidConstants: PIDConstants,
        private val aimPrecision: Precision<ScalarDimension> = Precision.Within(Scalar(0.02))
    ): () -> AngularVelocity? { // implements function type in order to be passed into rotation override acceptor
        private val pidController = PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD)

        override fun invoke(): AngularVelocity? {
            // if no targets are found, don't override rotation
            val bestTarget = visionSystem.bestTarget ?: return null

            // chargerlib extension function
            // scalar has to be used because Precision only works with kmeasure quantities,
            // so Scalar is basically a stand-in for a bare Double/unitless value.
            if (Scalar(bestTarget.tx).within(aimPrecision)){
                return AngularVelocity(0.0)
            }

            return AngularVelocity(pidController.calculate(bestTarget.tx, 0.0))
        }
    }
}
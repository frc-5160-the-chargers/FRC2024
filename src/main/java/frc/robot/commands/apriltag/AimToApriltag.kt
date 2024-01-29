package frc.robot.commands.apriltag

import com.batterystaple.kmeasure.quantities.Scalar
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.Alert
import frc.robot.constants.OPEN_LOOP_TRANSLATION_PID
import org.littletonrobotics.junction.Logger

val AIM_PRECISION = Precision.Within(Scalar(0.02))
private val noTargetFoundAlert = Alert.warning(text = "A command is attempting to aim to an apriltag, but none can be found.")


fun aimToApriltag(
    targetId: Int? = null,
    aimingPID: PIDConstants = OPEN_LOOP_TRANSLATION_PID,
    /**
     * A lambda that fetches the forward power that the drivetrain will drive for
     * while aiming PID is happening.
     */
    getDrivePower: () -> Double = {0.0},
    drivetrain: EncoderHolonomicDrivetrain,
    visionIO: AprilTagVisionPipeline,
): Command =
    buildCommand(name = "Aim to Apriltag", logIndividualCommands = true){
        val aimingController by getOnceDuringRun {
            SuperPIDController(
                aimingPID,
                getInput = { Scalar(visionIO.bestTarget?.tx ?: 0.0) },
                target = Scalar(0.0),
                outputRange = Scalar(-0.5)..Scalar(0.5)
            )
        }

        fun canFindTarget(): Boolean {
            val bestTarget = visionIO.bestTarget

            return if (bestTarget == null){
                noTargetFoundAlert.active = true
                false
            }else if (targetId != bestTarget.fiducialId ){
                Alert.warning(text =
                "An apriltag command can find an apriltag, " +
                        "but it does not match the id of $targetId. " +
                        "(Found id: " + bestTarget.fiducialId
                ).active = true
                false
            }else{
                true
            }
        }

        fun hasFinishedAiming(): Boolean {
            val aimingError = aimingController.error
            Logger.recordOutput("AimToAngle/aimingError", aimingError.siValue)
            return aimingError in AIM_PRECISION.allowableError
        }

        addRequirements(drivetrain)

        runOnce{
            visionIO.requireAndReset()
        }

        loopWhile({ canFindTarget() && !hasFinishedAiming() }){
            drivetrain.swerveDrive(
                xPower = getDrivePower(),
                yPower = aimingController.calculateOutput().siValue,
                rotationPower = 0.0
            )
        }

        runOnce{
            visionIO.removeRequirement()
            drivetrain.stop()
        }
    }
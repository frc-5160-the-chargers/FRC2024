@file:Suppress("unused")
package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Scalar
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.controls.pid.SuperPIDController
import frc.chargers.hardware.sensors.vision.AprilTagVisionPipeline
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.Precision
import frc.chargers.wpilibextensions.Alert
import frc.robot.constants.OPEN_LOOP_TRANSLATION_PID
import frc.robot.constants.PATHFIND_CONSTRAINTS
import frc.robot.hardware.subsystems.shooter.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull

private val noTargetFoundAlert = Alert.warning(text = "A command is attempting to aim to an apriltag, but none can be found.")
private val AIM_PRECISION = Precision.Within(Scalar(0.02))


enum class FieldLocation(
    val blueAllianceApriltagId: Int,
    val redAllianceApriltagId: Int,
    val shooterAngle: PivotAngle,
){
    SOURCE_LEFT(
        blueAllianceApriltagId = 2,
        redAllianceApriltagId = 10,
        shooterAngle = PivotAngle.SOURCE
    ), // tbd

    SOURCE_RIGHT(
        blueAllianceApriltagId = 1,
        redAllianceApriltagId = 9,
        shooterAngle = PivotAngle.SOURCE
    ), // tbd

    AMP(
        blueAllianceApriltagId = 6,
        redAllianceApriltagId = 5,
        shooterAngle = PivotAngle.AMP
    )
}

/**
 * Aims to a location on the field, and paths to there if applicable.
 */
fun aimToLocation(
    target: FieldLocation,
    path: PathPlannerPath? = null,

    drivetrain: EncoderHolonomicDrivetrain,
    apriltagVision: AprilTagVisionPipeline,
    shooter: Shooter,
): Command = buildCommand {
    addRequirements(drivetrain, shooter)

    val targetId = when (DriverStation.getAlliance().getOrNull()){
        DriverStation.Alliance.Blue, null -> target.blueAllianceApriltagId

        DriverStation.Alliance.Red -> target.redAllianceApriltagId
    }

    val aimingController by getOnceDuringRun {
        SuperPIDController(
            OPEN_LOOP_TRANSLATION_PID,
            getInput = { Scalar(apriltagVision.bestTarget?.tx ?: 0.0) },
            target = Scalar(0.0),
            outputRange = Scalar(-0.5)..Scalar(0.5)
        )
    }

    @AutoLogOutput(key = "AimToLocation/canFindTarget")
    fun canFindTarget(): Boolean {
        val bestTarget = apriltagVision.bestTarget

        return if (bestTarget == null){
            noTargetFoundAlert.active = true
            false
        }else if (targetId != bestTarget.fiducialId){
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

    @AutoLogOutput(key = "AimToLocation/hasFinishedAiming")
    fun hasFinishedAiming(): Boolean {
        val aimingError = aimingController.error
        Logger.recordOutput("AimToLocation/aimingError", aimingError.siValue)
        return aimingError in AIM_PRECISION.allowableError
    }


    runParallelUntilAllFinish {
        +shooter.setAngleCommand(target.shooterAngle)

        runSequentially{
            if (path != null){
                +AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS)
            }

            runOnce{
                apriltagVision.reset()
            }

            loopWhile({ canFindTarget() && !hasFinishedAiming() }){
                drivetrain.swerveDrive(
                    xPower = 0.0,
                    yPower = aimingController.calculateOutput().siValue,
                    rotationPower = 0.0
                )
            }

            runOnce{
                drivetrain.stop()
            }
        }

    }

}
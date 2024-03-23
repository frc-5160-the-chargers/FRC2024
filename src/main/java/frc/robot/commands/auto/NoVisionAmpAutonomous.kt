package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.commandbuilder.buildCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.utils.flipWhenNeeded
import frc.robot.commands.auto.components.AmpAutoComponent
import frc.robot.commands.auto.components.AmpAutoTaxiMode
import frc.robot.commands.auto.components.AutoStartingPose
import frc.robot.commands.followPathOptimal
import frc.robot.commands.runGroundIntake
import frc.robot.commands.passSerializedNote
import frc.robot.commands.shootInAmp
import frc.robot.hardware.subsystems.groundintake.GroundIntakeSerializer
import frc.robot.hardware.subsystems.pivot.Pivot
import frc.robot.hardware.subsystems.pivot.PivotAngle
import frc.robot.hardware.subsystems.shooter.Shooter

/**
 * An Amp autonomous command without vision cameras.
 */
fun noVisionAmpAutonomous(
    drivetrain: EncoderHolonomicDrivetrain,
    shooter: Shooter,
    pivot: Pivot,
    groundIntake: GroundIntakeSerializer,

    additionalComponents: List<AmpAutoComponent> = listOf(), // used to control further notes pursued.
    taxiMode: AmpAutoTaxiMode = AmpAutoTaxiMode.NO_TAXI
): Command = buildCommand {
    addRequirements(drivetrain, shooter, pivot, groundIntake)

    runOnce{
        drivetrain.poseEstimator.resetPose(
            AutoStartingPose.AMP_BLUE.flipWhenNeeded()
        )
    }

    +AutoBuilder.followPath(
        PathPlannerPath.fromPathFile("DriveToAmp")
    )

    +shootInAmp(shooter, pivot)


    for (autoComponent in additionalComponents){
        // starts ground intake a little before path
        +runGroundIntake(groundIntake, shooter, timeout = 0.5.seconds)

        runParallelUntilFirstCommandFinishes{
            // parallel #1
            runSequentially{
                +followPathOptimal(drivetrain, autoComponent.grabPath)

                waitFor(0.5.seconds)
            }

            // parallel #2
            +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

            // parallel #3
            +runGroundIntake(groundIntake, shooter)
        }

        when (autoComponent.type){
            AmpAutoComponent.Type.SCORE_NOTE -> {
                runParallelUntilFirstCommandFinishes{
                    +followPathOptimal(drivetrain, autoComponent.scorePath)

                    runSequentially{
                        +pivot.setAngleCommand(PivotAngle.GROUND_INTAKE_HANDOFF)

                        +passSerializedNote(groundIntake, shooter)
                    }
                }

                +shootInAmp(shooter, pivot)
            }

            AmpAutoComponent.Type.FERRY_NOTE -> {
                +followPathOptimal(drivetrain, autoComponent.scorePath)

                loopFor(1.seconds){
                    groundIntake.outtake()
                }

                runOnce{
                    groundIntake.setIdle()
                }
            }
        }
    }

    +pivot.setAngleCommand(PivotAngle.STOWED)

    when (taxiMode){
        AmpAutoTaxiMode.TAXI_SHORT -> {
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxiShort"))
        }

        AmpAutoTaxiMode.TAXI_LONG -> {
            +AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideTaxi"))
        }

        AmpAutoTaxiMode.NO_TAXI -> {}
    }
}
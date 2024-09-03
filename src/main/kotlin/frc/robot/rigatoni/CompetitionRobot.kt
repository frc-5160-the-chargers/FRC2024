package frc.robot.rigatoni

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.RunCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.subsystems.swervedrive.AimToAngleRotationOverride
import frc.robot.rigatoni.commands.AutoCommandManager
import frc.robot.rigatoni.commands.noteIntakeDriverAssist
import frc.robot.rigatoni.commands.passSerializedNote
import frc.robot.rigatoni.commands.shootInSpeaker
import frc.robot.rigatoni.inputdevices.DriverController
import frc.robot.rigatoni.inputdevices.OperatorInterface
import frc.robot.rigatoni.subsystems.Climber
import frc.robot.rigatoni.subsystems.GroundIntakeSerializer
import frc.robot.rigatoni.subsystems.NoteObserver
import frc.robot.rigatoni.subsystems.getDrivetrain
import frc.robot.rigatoni.subsystems.pivot.Pivot
import frc.robot.rigatoni.subsystems.pivot.PivotAngle
import frc.robot.rigatoni.subsystems.shooter.Shooter
import kotlin.jvm.optionals.getOrNull

/**
 * Our competition robot; rigatoni.
 */
@Suppress("unused")
class CompetitionRobot: ChargerRobot(){
    val gyro = ChargerNavX()
    val drivetrain = getDrivetrain(gyro)
    val pivot = Pivot()
    val groundIntake = GroundIntakeSerializer()
    val shooter = Shooter()
    val climber = Climber()
    val noteObserver = NoteObserver()

    val autoChooser = AutoCommandManager(drivetrain, shooter, groundIntake, pivot, noteObserver)

    override fun robotInit(){
        DriverController
        OperatorInterface

        DriverStation.silenceJoystickConnectionWarning(true)
        gyro.simHeadingSource = { drivetrain.heading }
        SmartDashboard.putData(
            "Power Distribution",
            PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        )

        setDefaultCommands()
        setButtonBindings()
        autonomous().whileTrue(
            Commands.deferredProxy{ autoChooser.selectedAuto }
        )
    }

    private fun setDefaultCommands(){
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(DriverController.swerveOutput)
        }

        shooter.setDefaultRunCommand{
            val speed = OperatorInterface.shooterSpeed
            if (speed > 0.0 && noteObserver.noteInRobot){
                shooter.setIdle()
            }else{
                shooter.setSpeed(speed)
            }
        }

        pivot.setDefaultRunCommand(endBehavior = { pivot.setIdle() }){
            setSpeed(OperatorInterface.pivotSpeed)
        }

        climber.setDefaultRunCommand{
            if (DriverController.climbersUpTrigger.asBoolean){
                moveLeftHook(1.0)
                moveRightHook(1.0)
            }else if (DriverController.climbersDownTrigger.asBoolean){
                moveLeftHook(-1.0)
                moveRightHook(-1.0)
            }else{
                setIdle()
            }
        }

        groundIntake.setDefaultRunCommand { setIdle() }
    }

    private fun setButtonBindings() {
        fun resetAimToAngle() = InstantCommand {
            drivetrain.removeRotationOverride()
        }

        fun targetAngle(heading: Angle) = InstantCommand {
            // used to make pivot side the front instead of the ground intake side
            val targetAngleOffset = 180.degrees

            val allianceAngleCompensation = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red){
                180.degrees
            } else {
                0.degrees
            }

            drivetrain.setRotationOverride(
                AimToAngleRotationOverride(
                    { heading + allianceAngleCompensation + targetAngleOffset },
                    ANGLE_TO_ROTATIONAL_VELOCITY_PID
                )
            )
        }

        DriverController.apply{
            pointNorthTrigger.onTrue(targetAngle(0.degrees)).onFalse(resetAimToAngle())

            pointEastTrigger.onTrue(targetAngle(-90.degrees)).onFalse(resetAimToAngle())

            pointSouthTrigger.onTrue(targetAngle(-180.degrees)).onFalse(resetAimToAngle())

            pointWestTrigger.onTrue(targetAngle(-270.degrees)).onFalse(resetAimToAngle())

            zeroHeadingTrigger.onTrue(InstantCommand{ gyro.zeroHeading(180.degrees) })

            driveToNoteAssistTrigger.whileTrue(noteIntakeDriverAssist(drivetrain, noteObserver))
        }

        // loopCommand and runOnceCommand are wrappers around RunCommand and InstantCommand
        // which allows for outer lambda block syntax
        OperatorInterface.apply{
            groundIntakeTrigger.whileTrue(
                RunCommand(groundIntake){ groundIntake.intake() }
            )

            groundOuttakeTrigger.whileTrue(
                RunCommand(groundIntake, shooter){
                    groundIntake.outtake()
                    shooter.setVoltage((-6).volts)
                }
            )

            passToShooterTrigger.whileTrue(passSerializedNote(noteObserver, groundIntake, shooter))

            spinUpShooterTrigger.whileTrue(
                RunCommand(shooter, pivot){
                    shooter.outtakeAtSpeakerSpeed()
                    pivot.setAngle(PivotAngle.SPEAKER)
                }
            )

            shootInSpeakerTrigger.whileTrue(
                shootInSpeaker(noteObserver, shooter, groundIntake, pivot, shooterSpinUpTime = 0.3.seconds)
            )

            // when only held, the buttons will just cause the pivot to PID to the appropriate position
            // interrupt behavior set as to prevent command scheduling conflicts
            ampPositionTrigger.whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.AMP)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            sourcePositionTrigger.whileTrue(
                pivot
                    .setAngleCommand(PivotAngle.SOURCE)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
            )

            stowPivotTrigger.whileTrue(pivot.setAngleCommand(PivotAngle.STOWED))

            y().whileTrue(
                RunCommand(shooter){
                    shooter.outtakeAtSpeakerSpeed()
                }
            )
        }
    }
}

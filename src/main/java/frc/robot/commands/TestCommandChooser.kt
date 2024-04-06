package frc.robot.commands

import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.chargers.commands.runOnceCommand
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.wpilibextensions.runAllTests
import frc.external.frc6328.MechanicalAdvantageFFCharacterization
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.subsystems.shooter.Shooter
import frc.robot.hardware.subsystems.vision.VisionManager
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

class TestCommandChooser(
    private val drivetrain: EncoderHolonomicDrivetrain,
    private val shooter: Shooter,
    private val visionManager: VisionManager? = null
) {
    private val sendableChooser = LoggedDashboardChooser<Command>("TestCommandOptions(Not Auto!!!!)")

    val selected: Command get() = sendableChooser.get() ?: Commands.none()

    init{
        sendableChooser.apply{
            addDefaultOption("Do Nothing", Commands.none())

            if (visionManager != null){
                addOption(
                    "Enable Vision Pose Estimation",
                    runOnceCommand{
                        visionManager.enableVisionPoseEstimation()
                        println("ALERT: VISION POSE ESTIMATION HAS BEEN ENABLED! POSES MIGHT BE OFF.")
                        println("ALERT: VISION POSE ESTIMATION HAS BEEN ENABLED! POSES MIGHT BE OFF.")
                        println("ALERT: VISION POSE ESTIMATION HAS BEEN ENABLED! POSES MIGHT BE OFF.")
                        println("ALERT: VISION POSE ESTIMATION HAS BEEN ENABLED! POSES MIGHT BE OFF.")
                        println("ALERT: VISION POSE ESTIMATION HAS BEEN ENABLED! POSES MIGHT BE OFF.")
                    }
                )
            }

            addOption(
                "Rumble With Regular HID test",
                Commands.startEnd(
                    { DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                    { DriverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) }
                ).withTimeout(2.0)
            )

            addOption(
                "Rumble With XboxController HID test",
                Commands.startEnd(
                    { DriverController.rumbleProcessor.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                    { DriverController.rumbleProcessor.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) }
                ).withTimeout(2.0)
            )

            addOption(
                "Rumble With Ps4Controller HID test",
                Commands.startEnd(
                    { DriverController.rumbleProcessor2.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                    { DriverController.rumbleProcessor2.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) }
                ).withTimeout(2.0)
            )

            addOption(
                "Drivetrain FF Characterization(Mechanical Advantage)",
                MechanicalAdvantageFFCharacterization(
                    drivetrain, true,
                    MechanicalAdvantageFFCharacterization.FeedForwardCharacterizationData("DrivetrainDataLeft"),
                    MechanicalAdvantageFFCharacterization.FeedForwardCharacterizationData("DrivetrainDataRight"),
                    { leftV, rightV ->
                        drivetrain.setDriveVoltages(
                            listOf(
                                leftV.ofUnit(volts),
                                rightV.ofUnit(volts),
                                leftV.ofUnit(volts),
                                rightV.ofUnit(volts),
                            )
                        )
                        drivetrain.setTurnDirections(
                            listOf(
                                0.degrees, 0.degrees, 0.degrees, 0.degrees
                            )
                        )
                    },
                    {
                        val allVelocities = drivetrain.moduleAngularVelocities
                        (allVelocities[0].siValue + allVelocities[2].siValue) / 2.0
                    },
                    {
                        val allVelocities = drivetrain.moduleAngularVelocities
                        (allVelocities[1].siValue + allVelocities[3].siValue) / 2.0
                    }
                )
            )

            addOption(
                "Shooter FF Characterization(Mechanical Advantage)",
                MechanicalAdvantageFFCharacterization(
                    shooter, false,
                    MechanicalAdvantageFFCharacterization.FeedForwardCharacterizationData("ShooterData"),
                    { voltage -> shooter.outtake(voltage) },
                    { shooter.angularVelocity.siValue },
                )
            )

            addOption(
                "Drivetrain FF Characterization(SysID)",
                drivetrain.getDriveSysIdRoutine().runAllTests()
            )
        }
    }
}
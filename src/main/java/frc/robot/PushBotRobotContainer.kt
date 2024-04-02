@file:Suppress("unused")
package frc.robot

import com.batterystaple.kmeasure.units.amps
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.chargers.commands.loopCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.subsystems.differentialdrive.sparkMaxDrivetrain

class PushBotRobotContainer: ChargerRobotContainer() {
    private val drivetrain = sparkMaxDrivetrain(
        topLeft = ChargerSparkMax(15){ inverted = true },
        topRight = ChargerSparkMax(7),
        bottomLeft = ChargerSparkMax(11){ inverted = true },
        bottomRight = ChargerSparkMax(23)
    ){
        smartCurrentLimit = SmartCurrentLimit(40.amps)
        voltageCompensationNominalVoltage = 12.volts
        openLoopRampRate = 48.0
        closedLoopRampRate = 48.0
    }
    
    private val xboxController = CommandXboxController(0)

    private val digitalOutputTest = DigitalOutput(5)

    init{
        drivetrain.setDefaultRunCommand {
            /*
            val precisionModePower = xboxController.leftTriggerAxis.mapBetweenRanges(0.0..1.0, 1.0..6.0)
            drivetrain.curvatureDrive(
                ChassisPowers(
                    MathUtil.applyDeadband(xboxController.leftY, .2) * precisionModePower,
                    0.0,
                    -MathUtil.applyDeadband(xboxController.rightX, .2) * precisionModePower
                )
            )
             */
            drivetrain.stop()
        }

        xboxController.a()
            .whileTrue(loopCommand{ digitalOutputTest.set(true) })
            .onFalse(loopCommand{ digitalOutputTest.set(false) })
    }


    override val autonomousCommand: Command
        get() = Commands.none()
}
package frc.robot

import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.constants.SwerveAzimuthControl
import frc.chargers.constants.SwerveControlData
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobotContainer
import frc.chargers.hardware.inputdevices.InputAxis
import frc.chargers.hardware.subsystems.swervedrive.EncoderHolonomicDrivetrain
import frc.chargers.hardware.subsystems.swervedrive.sparkMaxSwerveMotors
import frc.chargers.hardware.subsystems.swervedrive.swerveCANcoders
import frc.chargers.utils.math.equations.Polynomial
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.robot.hardware.inputdevices.Driver
import frc.robot.hardware.inputdevices.DriverController
import kotlin.jvm.optionals.getOrNull

/**
 * A defense sim robot container; with 2 robots that are controlled via 2 separate controllers.
 */
@Suppress("unused")
class DefenseSimulatorRobotContainer: ChargerRobotContainer() {
    // had to duplicate the driver controller stuff cuz we use singletons for those
    private object SecondaryDriverController: CommandXboxController(1){
        /* Top-Level constants */
        private const val DEFAULT_DEADBAND = 0.1
        private val DRIVER = Driver.NAYAN

        val shouldDisableFieldRelative: Boolean
            get() = start().asBoolean || back().asBoolean


        /* Private implementation */
        private val forwardAxis =
            InputAxis{ if (DRIVER.rightHanded) rightY else leftY }
                .applyDeadband(DEFAULT_DEADBAND)
                .invertWhen{ DriverStation.getAlliance().getOrNull() != DriverStation.Alliance.Red || shouldDisableFieldRelative }
                .applyMultiplier(0.6)
                .log("SecondaryDriverController/xPower")

        private val strafeAxis =
            InputAxis{ if (DRIVER.rightHanded) rightX else leftX }
                .applyDeadband(DEFAULT_DEADBAND)
                .invertWhen{ DriverStation.getAlliance().getOrNull() != DriverStation.Alliance.Red || shouldDisableFieldRelative }
                .applyMultiplier(0.6)
                .log("SecondaryDriverController/yPower")

        private val rotationAxis =
            InputAxis{ if (DRIVER.rightHanded) leftX else rightX }
                .applyDeadband(DEFAULT_DEADBAND)
                .square()
                .applyEquation(Polynomial(-0.1,0.0,-0.4,0.0))
                .log("SecondaryDriverController/rotationPower")

        private val precisionAxis =
            InputAxis{ leftTriggerAxis }
                .mapToRange(1.0..4.0)
                .withModifier{ if (it < 1.0 || it.isInfinite() || it.isNaN()) 1.0 else it }
                .withModifier{ 1.0 / it }
                .log("SecondaryDriverController/precisionPower")

        val swerveOutput: ChassisPowers get(){
            val scalar = precisionAxis()

            return ChassisPowers(
                xPower = forwardAxis() * scalar,
                yPower = strafeAxis() * scalar,
                rotationPower = rotationAxis() * scalar
            )
        }
    }

    init{
        require(RobotBase.isSimulation()){ "The defense simulator only works during sim." }
        DriverController
        SecondaryDriverController
    }

    // real motors aren't used; this is sim-only
    private val dummyTurnMotors = sparkMaxSwerveMotors(0,1,2,3)
    private val dummySwerveEncoders = swerveCANcoders(0,1,2,3, useAbsoluteSensor = false)
    private val dummyDriveMotors = sparkMaxSwerveMotors(4,5,6,7)


    private val standardHardwareData = SwerveHardwareData.mk4iL2(
        turnMotorType = DCMotor.getNEO(1),
        driveMotorType = DCMotor.getNEO(1),
        maxModuleSpeed = 4.5.meters / 1.seconds,
        trackWidth = 32.inches, wheelBase = 32.inches
    )

    private val standardControlData = SwerveControlData(
        azimuthControl = SwerveAzimuthControl.PID(
            PIDConstants(12.0,0,0.2),
        ),
        openLoopDiscretizationRate = 4.5,
        velocityPID = PIDConstants(0.2,0.0,0.0),
        velocityFF = if (RobotBase.isReal()){
            AngularMotorFFEquation(0.12117,0.13210)
        }else{
            AngularMotorFFEquation(0.0081299, 0.13396)
        },
        robotRotationPID = PIDConstants(1.3,0,0.1),
        robotTranslationPID = PIDConstants(0.3,0,0.03)
    )


    private val drivetrain1 = EncoderHolonomicDrivetrain(
        dummyTurnMotors, dummySwerveEncoders, dummyDriveMotors,
        standardHardwareData, standardControlData
    ).apply{
        // extension function in chargerlib
        setDefaultRunCommand{
            swerveDrive(
                DriverController.swerveOutput,
                fieldRelative = !DriverController.shouldDisableFieldRelative
            )
        }
    }

    private val drivetrain2 = EncoderHolonomicDrivetrain(
        dummyTurnMotors, dummySwerveEncoders, dummyDriveMotors,
        standardHardwareData, standardControlData
    ).apply{
        setDefaultRunCommand{
            swerveDrive(
                SecondaryDriverController.swerveOutput,
                fieldRelative = !SecondaryDriverController.shouldDisableFieldRelative
            )
        }
    }

    override val autonomousCommand: Command
        get() = Commands.idle()
}
package frc.robot

import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.grams
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.kilo
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.simulation.BasicMotorSim
import frc.chargers.hardware.subsystems.swervedrive.*
import frc.robot.inputdevices.DriverController


class Robot: ChargerRobot(){
    override val logToFileOnly = DriverStation.isFMSAttached()

    val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = SwerveData.generate{
            BasicMotorSim(DCMotor.getNEO(1), 0.00000871111.ofUnit(kilo.grams * (meters * meters)))
        },
        driveMotors = SwerveData.generate{
            BasicMotorSim(DCMotor.getKrakenX60(1), 0.00005444444.ofUnit(kilo.grams * (meters * meters)))
        },
        chassisConstants = SwerveChassisConstants(
            trackWidth = 27.inches,
            wheelBase = 27.inches
        ),
        moduleConstants = SwerveModuleConstants(
            useOnboardPID = false,
            turnMotorControlScheme = SwerveAzimuthControl.PID( PIDConstants(7.0,0,0) ),
            velocityPID = PIDConstants(0.2,0,0),
            velocityFF = AngularMotorFFEquation(0.0,0.13),
            wheelDiameter = 4.inches,
            turnGearRatio = 150.0 / 7.0,
            driveGearRatio = 6.75
        )
    )

    init{
        DriverController
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(
                DriverController.swerveOutput
            )
        }
        DriverStation.silenceJoystickConnectionWarning(true)
    }
}

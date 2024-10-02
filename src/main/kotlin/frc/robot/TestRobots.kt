package frc.robot

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.volts
import com.pathplanner.lib.util.PIDConstants
import frc.chargers.controls.feedforward.AngularMotorFeedforward
import frc.chargers.framework.ChargerRobot
import frc.chargers.framework.HorseLog.log
import frc.chargers.hardware.motorcontrol.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.sensors.encoders.ChargerCANcoder
import frc.chargers.hardware.subsystems.swervedrive.ModuleType
import frc.chargers.hardware.subsystems.swervedrive.SwerveConstants
import frc.chargers.hardware.subsystems.swervedrive.SwerveModule

class MotorTestingBot(private val motor: Motor): ChargerRobot() {
    init {
        motor.configure(
            gearRatio = 1.0, // Change this
            positionPID = PIDConstants(0.3, 0.0, 0.001),
            startingPosition = 0.degrees
        )
    }

    override fun robotPeriodic() {
        log("MotorPosition", motor.encoder.angularPosition)
    }

    override fun testPeriodic() {
        motor.setPositionSetpoint(90.degrees)
    }

    override fun autonomousPeriodic() {
        motor.voltageOut = 5.volts
    }

    override fun disabledInit() {
        motor.stop()
    }
}

class SwerveModuleTestingBot: ChargerRobot() {
    private val constants = SwerveConstants(
        moduleType = ModuleType.MK4iL2,
        trackWidth = 27.inches,
        wheelBase = 27.inches,
        azimuthPID = PIDConstants(7.0,0.0,0.0),
        azimuthPIDTolerance = 1.degrees,
        velocityPID = PIDConstants(0.05,0.0,0.0),
        velocityFF = AngularMotorFeedforward(0.0,0.13),
    )

    private val module = SwerveModule(
        "TestModule",
        ChargerSparkMax(12),
        ChargerCANcoder(44) - 0.621.radians,
        ChargerTalonFX(10),
        constants
    )

    override fun autonomousPeriodic() {
        module.setDirection(90.degrees)
    }

    override fun testPeriodic() {
        module.setDirection(-90.degrees)
    }
}
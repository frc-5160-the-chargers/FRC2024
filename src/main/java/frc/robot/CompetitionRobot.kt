package frc.robot

import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.controls.feedforward.AngularMotorFFEquation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.motorcontrol.rev.util.MotorData
import frc.chargers.hardware.motorcontrol.rev.util.PeriodicFrameConfig
import frc.chargers.hardware.motorcontrol.rev.util.SmartCurrentLimit
import frc.chargers.hardware.motorcontrol.simulation.MotorSim
import frc.chargers.hardware.sensors.encoders.ChargerCANcoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.sensors.imu.IMUSimulation
import frc.chargers.hardware.sensors.withOffset
import frc.chargers.hardware.subsystems.swervedrive.*
import frc.chargers.utils.Precision
import frc.robot.inputdevices.DriverController


class CompetitionRobot: ChargerRobot(){
    override val logToFileOnly = DriverStation.isFMSAttached()
    override val logFileName = "Testing.wpilog"

    val gyro = ChargerNavX()

    val drivetrain = EncoderHolonomicDrivetrain(
        turnMotors = if (isSimulation()){
            SwerveData.create{
                MotorSim(DCMotor.getNEO(1), moi = 0.0000087.ofUnit(kilo.grams * (meters * meters)))
            }
        }else{
            SwerveData(
                topLeft = ChargerSparkMax(DrivetrainID.TL_TURN),
                topRight = ChargerSparkMax(DrivetrainID.TR_TURN){ inverted = true },
                bottomLeft = ChargerSparkMax(DrivetrainID.BL_TURN),
                bottomRight = ChargerSparkMax(DrivetrainID.BR_TURN)
            ){ // configure lambda; configuration below are applied to all motors
                periodicFrameConfig = PeriodicFrameConfig.Optimized(utilizedData = listOf(MotorData.VELOCITY, MotorData.VOLTAGE, MotorData.CURRENT))
                smartCurrentLimit = SmartCurrentLimit(30.amps)
                voltageCompensationNominalVoltage = 12.volts
                openLoopRampRate = 48.0
                closedLoopRampRate = 48.0
            }
        },
        turnEncoders = if (isSimulation()){
            SwerveData.create{ null }
        }else{
            SwerveData(
                topLeft = ChargerCANcoder(DrivetrainID.TL_ENCODER).absolute.withOffset(0.621.radians),
                topRight = ChargerCANcoder(DrivetrainID.TR_ENCODER).absolute.withOffset(1.37.radians),
                bottomLeft = ChargerCANcoder(DrivetrainID.BL_ENCODER).absolute.withOffset(4.971.radians),
                bottomRight = ChargerCANcoder(DrivetrainID.BR_ENCODER).absolute.withOffset(6.243.radians)
            )
        },
        driveMotors = if (isSimulation()) {
            SwerveData.create{
                MotorSim(DCMotor.getKrakenX60(1), moi = 0.000054.ofUnit(kilo.grams * (meters * meters)))
            }
        }else{
            SwerveData(
                topLeft = ChargerTalonFX(DrivetrainID.TL_DRIVE),
                topRight = ChargerTalonFX(DrivetrainID.TR_DRIVE){ inverted = true },
                bottomLeft = ChargerTalonFX(DrivetrainID.BL_DRIVE),
                bottomRight = ChargerTalonFX(DrivetrainID.BR_DRIVE){ inverted = true },
            ){ // configure lambda; configuration below are applied to all motors
                statorCurrentLimitEnable = true
                statorCurrentLimit = 120.amps
                supplyCurrentLimitEnable = true
                supplyCurrentLimit = 60.amps
                neutralMode = NeutralModeValue.Brake
            }
        },
        chassisConstants = SwerveChassisConstants(
            trackWidth = 27.inches,
            wheelBase = 27.inches,
        ),
        moduleConstants = SwerveModuleConstants.mk4iL2(
            useOnboardPID = false,
            turnMotorControlScheme = SwerveAzimuthControl.PID(PIDConstants(7.0,0,0), Precision.Within(1.degrees)),
            velocityPID = PIDConstants(0.05,0,0),
            velocityFF = AngularMotorFFEquation(0.0,0.13),
        ),
        gyro = if (isSimulation()) null else gyro
    )



    override fun robotInit(){
        DriverController

        DriverStation.silenceJoystickConnectionWarning(true)

        IMUSimulation.configure(
            chassisSpeedsSupplier = { drivetrain.currentSpeeds },
            headingSupplier = { drivetrain.heading }
        )

        SmartDashboard.putData(
            "Power Distribution",
            PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        )

        setDefaultCommands()
        setButtonBindings()

        autonomous().whileTrue(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("RandomStuff"))
        )
    }

    override fun robotPeriodic() {
        super.robotPeriodic()
        gyro.broadcastOrientationForMegaTag2("Limelight2Main")
    }

    private fun setDefaultCommands(){
        drivetrain.setDefaultRunCommand{
            drivetrain.swerveDrive(DriverController.swerveOutput)
        }
    }

    private fun setButtonBindings() {

    }
}

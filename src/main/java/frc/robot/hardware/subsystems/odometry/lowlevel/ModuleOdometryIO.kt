package frc.robot.hardware.subsystems.odometry.lowlevel

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.rotations
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.REVLibError
import edu.wpi.first.wpilibj.Timer
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.motorcontrol.ctre.ChargerTalonFX
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.robot.ODOMETRY_UPDATE_FREQUENCY_HZ
import frc.robot.hardware.subsystems.odometry.OdometryThread
import org.littletonrobotics.junction.Logger
import java.util.*
import kotlin.math.roundToInt


/**
 * Handles multi-threaded odometry for a single swerve module.
 */
class ModuleOdometryIO(
    moduleName: String,
    // all wrappers inherit their base class(CANSparkMax)
    private val hardwareData: SwerveHardwareData,
    private val turnMotor: ChargerSparkMax,
    private val driveMotor: ChargerTalonFX,
    absoluteEncoder: PositionEncoder,
) {
    private val wheelRadius = hardwareData.wheelDiameter / 2.0


    // handles logging and replay for wheel odometry of each module
    private val logInputs = LoggableInputsProvider(
        "MultiThreadedOdometry/$moduleName",
        runBeforeInputUpdate = OdometryThread.ODOMETRY_LOCK::lock,
        runAfterInputUpdate = OdometryThread.ODOMETRY_LOCK::unlock
    )

    private val turnEncoder = turnMotor.getEncoder()
    private val drivePositionSignal = driveMotor.position.apply{
        setUpdateFrequency(ODOMETRY_UPDATE_FREQUENCY_HZ)
    }

    private val wheelPositionsQueue = OdometryThread.getInstance().registerSignal{
        val isValid = drivePositionSignal.refresh().status.isOK
        if (isValid){
            OptionalDouble.of(drivePositionSignal.valueAsDouble)
        }else{
            OptionalDouble.empty()
        }
    }

    private val wheelDirectionsQueue = OdometryThread.getInstance().registerSignal{
        val value = turnEncoder.position
        if (turnMotor.lastError == REVLibError.kOk) {
            OptionalDouble.of(value)
        } else {
            OptionalDouble.empty()
        }
    }

    private val wheelPositionOffset =
        (driveMotor.position.value.ofUnit(rotations)
                / hardwareData.driveGearRatio
                * wheelRadius // wheel radius; multiplying will get distance
        ).also{ println("Kraken drive offset: $it") }

    private val wheelDirectionOffset = (
            (turnEncoder.position.ofUnit(rotations) / hardwareData.turnGearRatio)
                    - absoluteEncoder.angularPosition
            ).also{ println("Spark max turn offset: $it") }

    // initialies previous wheel positions and directions to 0
    private var previousWheelPosition = Distance(0.0)
    private var previousWheelDirection = Angle(0.0)

    fun burnMotorFlashes(){
        turnMotor.burnFlash()
    }

    init{
        println("kStatus2(turn motor): " + turnMotor.setPeriodicFramePeriod(
            CANSparkLowLevel.PeriodicFrame.kStatus2, (1000.0 / ODOMETRY_UPDATE_FREQUENCY_HZ).roundToInt()
        ))
    }


    val wheelPositions by logInputs.quantityList{
        wheelPositionsQueue
            .stream()
            // transforms Double values into kmeasure Angles, then Distances(by multiplying by wheelRadius)
            .map{
                (it.ofUnit(rotations) / hardwareData.driveGearRatio * wheelRadius) - wheelPositionOffset
            }
            .toList()
            // filters invalid values; this can include NaNs, infinity values, or values that deviate too much from the previous value
            .filter{ value: Distance ->
                var isValid = true

                if (
                    value.siValue == Double.POSITIVE_INFINITY ||
                    value.siValue == Double.NEGATIVE_INFINITY ||
                    value.siValue.isNaN() ||
                    // absolute value overload for kmeasure quantities
                    abs( value - previousWheelPosition ) > 0.5.meters
                ){
                    isValid = false
                    recordDriveMotorBadReading()
                }else{
                    previousWheelPosition = value
                }

                return@filter isValid
            }
            // finally, clears the queue
            .also{ wheelPositionsQueue.clear() }
    }


    val wheelDirections by logInputs.quantityList{
        wheelDirectionsQueue
            .stream()
            // transforms Double values into kmeasure Angles, then Distances(by multiplying by wheelRadius)
            .map{
                (it.ofUnit(rotations) / hardwareData.driveGearRatio) - wheelDirectionOffset
            }
            .toList()
            // filters invalid values; this can include NaNs, infinity values, or values that deviate too much
            .filter{ value: Angle ->
                var isValid = true
                if (
                    value.siValue == Double.POSITIVE_INFINITY ||
                    value.siValue == Double.NEGATIVE_INFINITY ||
                    value.siValue.isNaN() ||
                    // absolute value overload for kmeasure quantities
                    abs( value - previousWheelDirection ) > 180.degrees
                ){
                    isValid = false
                    recordTurnMotorBadReading()
                }else{
                    previousWheelDirection = value
                }
                return@filter isValid
            }
            // finally, clears the queue
            .also{ wheelDirectionsQueue.clear() }
    }



    // companion object tracks bad readings and logs them
    companion object{
        private fun recordTurnMotorBadReading(){
            turnMotorBadReadingTimestamps.add(Timer.getFPGATimestamp())
        }

        private fun recordDriveMotorBadReading(){
            driveMotorBadReadingTimestamps.add(Timer.getFPGATimestamp())
        }

        private val driveMotorBadReadingTimestamps = mutableListOf<Double>()
        private val turnMotorBadReadingTimestamps = mutableListOf<Double>()

        init{
            ChargerRobot.runPeriodically {
                Logger.recordOutput(
                    "ThreadedPoseEstimator/driveMotorBadReadingTimestamps",
                    driveMotorBadReadingTimestamps.toDoubleArray()
                )

                Logger.recordOutput(
                    "ThreadedPoseEstimator/turnMotorBadReadingTimestamps",
                    turnMotorBadReadingTimestamps.toDoubleArray()
                )
            }
        }
    }
}
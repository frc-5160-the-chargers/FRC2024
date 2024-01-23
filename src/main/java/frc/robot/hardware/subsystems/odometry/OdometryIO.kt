@file:Suppress("unused")
package frc.robot.hardware.subsystems.odometry

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.rotations
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.wpilibj.RobotBase
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.constants.SwerveHardwareData
import frc.chargers.hardware.motorcontrol.rev.ChargerSparkMax
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.hardware.sensors.imu.ChargerNavX
import frc.chargers.hardware.subsystems.swervedrive.SwerveEncoders
import frc.chargers.hardware.subsystems.swervedrive.SwerveMotors
import frc.chargers.utils.math.equations.epsilonEquals
import frc.chargers.wpilibextensions.delay
import frc.robot.constants.ODOMETRY_UPDATE_FREQUENCY_HZ
import java.util.*
import kotlin.math.roundToInt

@JvmField
val OdometryLog = LoggableInputsProvider(
    "PoseData",
    updateInputs = RobotBase.isReal(), // if robot is simulation, don't update inputs.
    runBeforeInputUpdate = OdometryThread.ODOMETRY_LOCK::lock,
    runAfterInputUpdate = OdometryThread.ODOMETRY_LOCK::unlock
)

class OdometryIO(
    // all wrappers inherit their base class(CANSparkMax)
    val hardwareData: SwerveHardwareData,
    turnMotors: SwerveMotors<ChargerSparkMax>,
    turnEncoders: SwerveEncoders<PositionEncoder>,
    driveMotors: SwerveMotors<ChargerSparkMax>,
    gyro: ChargerNavX
){

    init{
        require(gyro.ahrs.requestedUpdateRate.toDouble() epsilonEquals ODOMETRY_UPDATE_FREQUENCY_HZ){
            "The NavX update rate is incorrect. Current update rate: " + gyro.ahrs.requestedUpdateRate.toDouble() + ", Needed rate: " + ODOMETRY_UPDATE_FREQUENCY_HZ
        }

        driveMotors.forEach{
            println("Status: " + it.setPeriodicFramePeriod(
                CANSparkLowLevel.PeriodicFrame.kStatus2, (1000.0 / ODOMETRY_UPDATE_FREQUENCY_HZ).roundToInt()
            ))
        }


        turnMotors.forEach{
            println("Status: " + it.setPeriodicFramePeriod(
                CANSparkLowLevel.PeriodicFrame.kStatus2, (1000.0 / ODOMETRY_UPDATE_FREQUENCY_HZ).roundToInt()
            ))
        }

        delay(0.02.seconds)

        driveMotors.forEach{ it.burnFlash() }
        turnMotors.forEach{ it.burnFlash() }
    }

    private val gyroReadingsQueue = OdometryThread.getInstance().registerSignal{ gyro.heading.inUnit(degrees) }

    private val wheelPositionQueueTL = OdometryThread.getInstance().registerSignal(driveMotors.topLeft.getEncoder()::getPosition)
    private val wheelPositionQueueTR = OdometryThread.getInstance().registerSignal(driveMotors.topRight.getEncoder()::getPosition)
    private val wheelPositionQueueBL = OdometryThread.getInstance().registerSignal(driveMotors.bottomLeft.getEncoder()::getPosition)
    private val wheelPositionQueueBR = OdometryThread.getInstance().registerSignal(driveMotors.bottomRight.getEncoder()::getPosition)

    private val wheelDirectionQueueTL = OdometryThread.getInstance().registerSignal(turnMotors.topLeft.getEncoder()::getPosition)
    private val wheelDirectionQueueTR = OdometryThread.getInstance().registerSignal(turnMotors.topRight.getEncoder()::getPosition)
    private val wheelDirectionQueueBL = OdometryThread.getInstance().registerSignal(turnMotors.bottomLeft.getEncoder()::getPosition)
    private val wheelDirectionQueueBR = OdometryThread.getInstance().registerSignal(turnMotors.bottomRight.getEncoder()::getPosition)

    private fun ChargerSparkMax.fetchDriveOffset(): Angle =
        (this.encoder.angularPosition / hardwareData.driveGearRatio).also{ println("Spark max drive offset: $it") }

    private fun ChargerSparkMax.fetchTurnOffset(encoder: PositionEncoder): Angle =
        ((this.encoder.angularPosition / hardwareData.turnGearRatio) - encoder.angularPosition).also{ println("Spark max turn offset: $it") }

    private val topLeftWheelPositionOffset = driveMotors.topLeft.fetchDriveOffset()
    private val topRightWheelPositionOffset = driveMotors.topRight.fetchDriveOffset()
    private val bottomLeftWheelPositionOffset = driveMotors.bottomLeft.fetchDriveOffset()
    private val bottomRightWheelPositionOffset = driveMotors.bottomRight.fetchDriveOffset()

    private val topLeftDirectionOffset = turnMotors.topLeft.fetchTurnOffset(turnEncoders.topLeft)
    private val topRightDirectionOffset = turnMotors.topRight.fetchTurnOffset(turnEncoders.topRight)
    private val bottomLeftDirectionOffset = turnMotors.bottomLeft.fetchTurnOffset(turnEncoders.bottomLeft)
    private val bottomRightDirectionOffset = turnMotors.bottomRight.fetchTurnOffset(turnEncoders.bottomRight)



    private fun Queue<Double>.asDrivePositionList(): List<Angle> =
        this.stream()
            .map{ it.ofUnit(rotations) / hardwareData.driveGearRatio }
            .toList()
            .also{ this.clear() } // clears the queue; not the list

    private fun Queue<Double>.asTurnPositionList(): List<Angle> =
        this.stream()
            .map{ it.ofUnit(rotations) / hardwareData.turnGearRatio }
            .toList()
            .also{ this.clear() } // clears the queue; not the list

    
    
    
    fun emptyQueues(){
        gyroReadingsQueue.clear()
        
        wheelPositionQueueTL.clear()
        wheelPositionQueueTR.clear()
        wheelPositionQueueBL.clear()
        wheelPositionQueueBR.clear()


        wheelDirectionQueueTL.clear()
        wheelDirectionQueueTR.clear()
        wheelDirectionQueueBL.clear()
        wheelDirectionQueueBR.clear()
    }
    
    

    val gyroHeadings by OdometryLog.quantityList{
        gyroReadingsQueue.stream().map{ it.ofUnit(degrees) }.toList().also{ gyroReadingsQueue.clear() }
    }
    

    val topLeftWheelPositions by OdometryLog.quantityList{
        wheelPositionQueueTL.asDrivePositionList().map { it - topLeftWheelPositionOffset }
    }

    val topRightWheelPositions by OdometryLog.quantityList{
        wheelPositionQueueTR.asDrivePositionList().map { it - topRightWheelPositionOffset }
    }

    val bottomLeftWheelPositions by OdometryLog.quantityList{
        wheelPositionQueueBL.asDrivePositionList().map { it - bottomLeftWheelPositionOffset }
    }

    val bottomRightWheelPositions by OdometryLog.quantityList{
        wheelPositionQueueBR.asDrivePositionList().map { it - bottomRightWheelPositionOffset }
    }


    
    val topLeftWheelDirections by OdometryLog.quantityList{
        wheelDirectionQueueTL.asTurnPositionList().map { it - topLeftDirectionOffset }
    }

    val topRightWheelDirections by OdometryLog.quantityList{
        wheelDirectionQueueTR.asTurnPositionList().map { it - topRightDirectionOffset }
    }

    val bottomLeftWheelDirections by OdometryLog.quantityList{
        wheelDirectionQueueBL.asTurnPositionList().map { it - bottomLeftDirectionOffset }
    }

    val bottomRightWheelDirections by OdometryLog.quantityList{
        wheelDirectionQueueBR.asTurnPositionList().map { it - bottomRightDirectionOffset }
    }
}
package frc.robot.rigatoni.subsystems

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.rigatoni.ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
import monologue.Annotations.Log
import monologue.Logged
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils

private const val SHOOTER_SENSOR_ID = 9
private const val GROUND_INTAKE_SENSOR_ID = 1

private val CAM_HEIGHT = 10.inches
private val CAM_PITCH = 37.degrees

sealed interface NoteState {
    data object None: NoteState
    data class Detected(val tx: Double, val distanceToNote: Distance): NoteState
    data object InSerializer: NoteState
    data object InShooter: NoteState
}

class NoteObserver: SubsystemBase(), Logged {
    private val groundIntakeSensor: DigitalInput? = null
    private val shooterSensor = DigitalInput(SHOOTER_SENSOR_ID)
    private val noteDetectorCamera: PhotonCamera? = if (isSimulation()) null else PhotonCamera("MLWebcam")

    var state: NoteState = NoteState.None
        private set(value) {
            field = value
            if (value is NoteState.Detected) {
                log("NoteObserver/Detection/IsPresent", true)
                log("NoteObserver/Detection/TX", value.tx)
                log("NoteObserver/Detection/DistanceToNoteMeters", value.distanceToNote.inUnit(meters))
            }else{
                log("NoteObserver/Detection/IsPresent", false)
                log("NoteObserver/Detection/TX", 0.0)
                log("NoteObserver/Detection/DistanceToNoteMeters", 0.0)
            }
            log("NoteObserver/State", value.toString())
        }

    @get:Log val noteInRobot get() = state == NoteState.InSerializer || state == NoteState.InShooter
    @get:Log val noteFound get() = state is NoteState.Detected

    @Log.Once val hasGroundIntakeSensor = groundIntakeSensor != null && RobotBase.isReal()
    @Log.Once val hasCamera = noteDetectorCamera != null && RobotBase.isReal()

    /** Whether note pursuit(via vision targeting) should start or not. */
    val shouldStartPursuit: Boolean get() {
        val currentState = this.state
        return currentState is NoteState.Detected &&
                currentState.distanceToNote <= ACCEPTABLE_DISTANCE_BEFORE_NOTE_INTAKE
    }

    override fun periodic() {
        if (isSimulation()){
            state = NoteState.None
        }else if (groundIntakeSensor != null && groundIntakeSensor.get() && !noteInRobot){
            state = NoteState.InSerializer
        }else if (!shooterSensor.get()){
            state = NoteState.InShooter
        }else{
            if (noteDetectorCamera == null){
                state = NoteState.None
                return
            }
            val camResult = noteDetectorCamera.latestResult
            if (camResult.hasTargets() && !noteInRobot){
                val estimatedDistance = PhotonUtils.calculateDistanceToTargetMeters(
                    CAM_HEIGHT.inUnit(meters),
                    0.0,
                    CAM_PITCH.inUnit(radians),
                    camResult.bestTarget.pitch
                )
                state = NoteState.Detected(
                    tx = camResult.bestTarget.yaw,
                    distanceToNote = estimatedDistance.ofUnit(meters)
                )
            }else{
                state = NoteState.None
            }
        }
    }
}
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
import frc.chargers.framework.SuperSubsystem
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils

private const val SHOOTER_SENSOR_ID = 9
private const val GROUND_INTAKE_SENSOR_ID = 1

private val CAM_HEIGHT = 10.inches
private val CAM_PITCH = 37.degrees

class NoteObserver: SuperSubsystem("NoteObserver") {
    sealed class State {
        data object NoNote: State()

        data class NoteDetected(val tx: Double, val distanceToNote: Distance): State()

        data object NoteInSerializer: State()

        data object NoteInShooter: State()
    }

    private val groundIntakeSensor: DigitalInput? = DigitalInput(GROUND_INTAKE_SENSOR_ID)
    private val shooterSensor = DigitalInput(SHOOTER_SENSOR_ID)
    private val noteDetectorCamera: PhotonCamera? = if (isSimulation()) null else PhotonCamera("MLWebcam")

    var state: State = State.NoNote
        private set(value) {
            field = value

            if (value is State.NoteDetected) {
                log("NoteDetection/IsPresent", true)
                log("NoteDetection/TX", value.tx)
                log("NoteDetection/DistanceToNoteMeters", value.distanceToNote.inUnit(meters))
            }else{
                log("NoteDetection/IsPresent", false)
                log("NoteDetection/TX", 0.0)
                log("NoteDetection/DistanceToNoteMeters", 0.0)
            }

            log("CurrentState", state.toString())
        }

    val noteInRobot: Boolean by logged{ state == State.NoteInSerializer || state == State.NoteInShooter }

    val hasGroundIntakeSensor: Boolean by logged(groundIntakeSensor != null && RobotBase.isReal())

    val hasCamera: Boolean by logged(noteDetectorCamera != null)

    override fun periodic() {
        if (isSimulation()){
            state = State.NoNote
            return
        }

        if (groundIntakeSensor != null && groundIntakeSensor.get() && !noteInRobot){
            state = State.NoteInSerializer
        }else if (shooterSensor.get() && state != State.NoteInShooter){
            state = State.NoteInShooter
        }else{
            if (noteDetectorCamera == null){
                state = State.NoNote
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

                state = State.NoteDetected(
                    tx = camResult.bestTarget.yaw,
                    distanceToNote = estimatedDistance.ofUnit(meters)
                )
            }else{
                state = State.NoNote
            }
        }
    }
}
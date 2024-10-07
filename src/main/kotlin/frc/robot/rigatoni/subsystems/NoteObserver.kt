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
import frc.chargers.framework.HorseLog.log
import frc.chargers.framework.logged
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

class NoteObserver: SubsystemBase() {
    private val groundIntakeSensor: DigitalInput? = DigitalInput(GROUND_INTAKE_SENSOR_ID)
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

    val noteInRobot: Boolean by logged{ state == NoteState.InSerializer || state == NoteState.InShooter }

    val noteFound: Boolean by logged{ state is NoteState.Detected }

    val hasGroundIntakeSensor: Boolean by logged(groundIntakeSensor != null && RobotBase.isReal())

    val hasCamera: Boolean by logged(noteDetectorCamera != null && RobotBase.isReal())

    override fun periodic() {
        if (isSimulation()){
            state = NoteState.None
        }else if (groundIntakeSensor != null && groundIntakeSensor.get() && !noteInRobot){
            state = NoteState.InSerializer
        }else if (shooterSensor.get() && state != NoteState.InShooter){
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
@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision.limelight

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.*
import frc.external.limelight.LimelightHelpers.*
import frc.chargers.advantagekitextensions.LoggableInputsProvider
import frc.chargers.framework.ChargerRobot
import frc.chargers.hardware.sensors.VisionPoseSupplier
import frc.chargers.hardware.sensors.vision.*
import frc.chargers.utils.Measurement
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import org.littletonrobotics.junction.Logger


/**
 * A wrapper around the Limelight, with AdvantageKit support,
 * optimized for usage within ChargerLib.
 */
public class ChargerLimelight(
    @JvmField public val name: String = "",
    useJsonDump: Boolean = false,
    public val robotToCamera: UnitTransform3d
){
    // used to manage requirements for the limelight.
    private var required: Boolean = false

    // Limelight Helpers results
    private var jsonResults: Results = Results().apply{ valid = false }

    private val allIndexes: MutableList<Int> = mutableListOf()

    private fun ensureIndexValid(index: Int){
        require (index !in allIndexes){ "There is already a Limelight pipeline with index $index." }
        require (index in 0..9){ "Your Limelight pipeline's index($index) is out of range." }
        allIndexes.add(index)
    }

    private fun updateJson(){
        if (useJsonDump && isReal() && getTV(name)){
            jsonResults = getLatestResults(name).targetingResults
        }else{
            jsonResults.valid = false
        }
    }

    private fun hasTargets() =
        if (useJsonDump) jsonResults.valid else getTV(name)


    /**
     * Determines whether to fetch the full JSON dump from the limelight every loop.
     * If the json dump is not fetched, direct NetworkTables data is used instead;
     * this prevents the camera from detecting multiple targets.
     */
    public var useJsonDump: Boolean = false
        set(shouldEnable){
            if (shouldEnable){
                ChargerRobot.runPeriodically(addToFront = true, ::updateJson)
            }else{
                ChargerRobot.removeFromLoop(::updateJson)
                jsonResults.valid = false
            }
            field = shouldEnable
        }

    init{
        this.useJsonDump = useJsonDump
        limelights.add(this)
        ChargerRobot.runPeriodically{
            Logger.recordOutput("Limelight/pipelineIndex", getCurrentPipelineIndex(name))
        }
    }

    public companion object{
        private val limelights: MutableList<ChargerLimelight> = mutableListOf()

        /**
         * Enables JSON dump for all limelights.
         *
         * @see ChargerLimelight.useJsonDump
         */
        public fun enableJsonDumpForAll(){
            limelights.forEach{
                it.useJsonDump = true
            }
        }

        /**
         * Disables JSON dump for all limelights.
         *
         * @see ChargerLimelight.useJsonDump
         */
        public fun disableJsonDumpForAll(){
            limelights.forEach{
                it.useJsonDump = false
            }
        }
    }


    /**
     * Represents a Limelight pipeline that can detect AprilTags,
     * and report the pose of the robot.
     */
    public inner class AprilTagPipeline(
        index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider,
        usePoseEstimation: Boolean
    ): LimelightPipeline<VisionTarget.AprilTag>(index), VisionPoseSupplier {

        override val visionTargets: List<VisionTarget.AprilTag>
            by logInputs.valueList(default = VisionTarget.AprilTag.Dummy){
                if (isSimulation() || !hasTargets() || getCurrentPipelineIndex(name).toInt() != index) {
                    return@valueList listOf()
                }

                if (useJsonDump){
                    val latency = jsonResults.latency_capture.ofUnit(milli.seconds)
                    return@valueList jsonResults.targets_Fiducials.map{
                        VisionTarget.AprilTag(
                            timestamp = fpgaTimestamp() - latency,
                            tx = it.tx,
                            ty = it.ty,
                            areaPercent = it.ta,
                            fiducialId = it.fiducialID.toInt(),
                            // converts it to a UnitTransform3d.
                            targetTransformFromCam = it.targetPose_CameraSpace.ofUnit(meters) - UnitPose3d()
                        )
                    }
                }else{
                    val latency = (getLatency_Capture(name) + getLatency_Pipeline(name)).ofUnit(milli.seconds)
                    return@valueList listOf(
                        VisionTarget.AprilTag(
                            fpgaTimestamp() - latency,
                            getTX(name),
                            getTY(name),
                            getTA(name),
                            getFiducialID(name).toInt(),
                            getTargetPose3d_CameraSpace(name).ofUnit(meters) - UnitPose3d()
                        )
                    )
                }
            }


        override val cameraYaw: Angle
            get() = Angle(robotToCamera.rotation.z)

        override val robotPoseEstimates: List<Measurement<UnitPose2d>>
            by logInputs.valueList(default = Measurement(UnitPose2d(), Time(0.0))){
                if (!usePoseEstimation || isSimulation() || !hasTargets() || getCurrentPipelineIndex(name).toInt() != index) {
                    return@valueList listOf()
                }

                val allianceColor: DriverStation.Alliance =
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)

                val poseArray: DoubleArray

                when(allianceColor) {
                    DriverStation.Alliance.Blue -> {
                        poseArray = if (useJsonDump){
                            jsonResults.botpose_wpiblue
                        }else{
                            getBotPose_wpiBlue(name)
                        }
                    }

                    DriverStation.Alliance.Red -> {
                        poseArray = if (useJsonDump){
                            jsonResults.botpose_wpired
                        }else{
                            getBotPose_wpiRed(name)
                        }
                    }
                }

                return@valueList listOf(
                    Measurement(
                        value = UnitPose2d(
                            poseArray[0].ofUnit(meters),
                            poseArray[1].ofUnit(meters),
                            poseArray[5].ofUnit(degrees)
                        ),
                        timestamp = fpgaTimestamp() - (poseArray[6].ofUnit(milli.seconds))
                    )
                )
            }
    }

    /**
     * Represents a Limelight pipeline which can detect objects.
     *
     * The method used to detect objects is agnostic; some options include machine learning and color pipelines.
     */
    public inner class ObjectPipeline(
        index: Int,
        private val isMachineLearning: Boolean,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): LimelightPipeline<VisionTarget.Object>(index){

        override val visionTargets: List<VisionTarget.Object>
            by logInputs.valueList<VisionTarget.Object>(default = VisionTarget.Object.Dummy) {
                if (isSimulation() || !hasTargets() || getCurrentPipelineIndex(name).toInt() != index) {
                    return@valueList listOf()
                }

                val latency: Time
                if (useJsonDump){
                    latency = jsonResults.latency_capture.ofUnit(milli.seconds)
                    return@valueList jsonResults.targets_Detector.map{
                        VisionTarget.Object(
                            timestamp = fpgaTimestamp() - latency,
                            tx = it.tx,
                            ty = it.ty,
                            areaPercent = it.ta,
                            classId = if (isMachineLearning) it.classID.toString() else null
                        )
                    }
                }else{
                    latency = (getLatency_Capture(name) + getLatency_Pipeline(name)).ofUnit(milli.seconds)
                    return@valueList listOf(
                        VisionTarget.Object(
                            fpgaTimestamp() - latency,
                            getTX(name),
                            getTY(name),
                            getTA(name),
                            if (isMachineLearning) getNeuralClassID(name).toString() else null
                        )
                    )
                }
            }
    }


    /**
     * Represents a base pipeline for a limelight.
     */
    abstract inner class LimelightPipeline<T: VisionTarget>(
        public val index: Int
    ): VisionPipeline<T> {
        init{
            ensureIndexValid(index)
            // if there is only 1 current index; reset the pipeline as the camera is likely only running on 1 index;
            if (allIndexes.size < 1){
                reset()
            }
        }

        final override fun reset(){
            if (getCurrentPipelineIndex(name).toInt() != index){
                setPipelineIndex(name, index)
                println("Limelight with name $name has had it's pipeline reset to $index")
            }else{
                println("Limelight with name $name is already on index $index.")
            }
        }

        override val cameraConstants = VisionCameraConstants(
            "Limelight " + if (this@ChargerLimelight.name == "") "(No Name)" else this@ChargerLimelight.name,
            this@ChargerLimelight.robotToCamera
        )
    }
}
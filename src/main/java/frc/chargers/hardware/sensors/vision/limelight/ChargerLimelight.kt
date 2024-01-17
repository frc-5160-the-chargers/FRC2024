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
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d


/**
 * A wrapper around the Limelight, optimized for usage within ChargerLib.
 */
public class ChargerLimelight(
    @JvmField public val name: String = "limelight",
    useJsonDump: Boolean = false,
    public val lensHeight: Distance,
    public val mountAngle: Angle,
    public val defaultDriverStationIfUnavailable: DriverStation.Alliance = DriverStation.Alliance.Blue,
){
    // used to manage requirements for the limelight.
    private var required: Boolean = false

    // Limelight Helpers results
    private var jsonResults: Results = Results().apply{ valid = false }

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



    public inner class ApriltagPipeline(
        override val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        private val logInputs: LoggableInputsProvider
    ): VisionPipeline<VisionTarget.AprilTag> {
        init{
            if (index < 0 || index > 9){
                error("Your pipeline's ID is out of range.")
            }
            reset()
        }

        override fun reset(){
            setPipelineIndex(name,index)
            println("Limelight with name $name has had it's pipeline reset to $index")
        }

        override val visionData: VisionData<VisionTarget.AprilTag>?
            by logInputs.nullableValue(default = emptyAprilTagVisionData()){ getApriltagData() }

        private fun getApriltagData(): VisionData<VisionTarget.AprilTag>?{
            if (isSimulation() || !hasTargets()) {
                return null
            }else if (getCurrentPipelineIndex(name).toInt() != index){
                println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                return null
            }

            if (useJsonDump){
                val allTargets = jsonResults.targets_Fiducials.map{
                    VisionTarget.AprilTag(
                        tx = it.tx,
                        ty = it.ty,
                        areaPercent = it.ta,
                        id = it.fiducialID.toInt(),
                        // converts it to a UnitTransform3d.
                        targetTransformFromCam = it.targetPose_CameraSpace.ofUnit(meters) - UnitPose3d()
                    )
                }.toMutableList()
                val bestTarget = allTargets.removeAt(0)
                val latency = jsonResults.latency_capture.ofUnit(milli.seconds)

                return VisionData(fpgaTimestamp() - latency, bestTarget, allTargets)
            }else{
                val latency = (getLatency_Capture(name) + getLatency_Pipeline(name)).ofUnit(milli.seconds)
                return VisionData(
                    fpgaTimestamp() - latency,
                    VisionTarget.AprilTag(
                        getTX(name),
                        getTY(name),
                        getTA(name),
                        getFiducialID(name).toInt(),
                        getTargetPose3d_CameraSpace(name).ofUnit(meters) - UnitPose3d()
                    )
                )
            }
        }

        override val lensHeight: Distance = this@ChargerLimelight.lensHeight

        override val mountAngle: Angle = this@ChargerLimelight.mountAngle

        override fun require(){
            if (required){
                error("A Limelight with name '$name' has been required in 2 different places. \n " +
                        "Make sure to call pipeline.isRequired = false at the end of all commands!"
                )
            }
            required = true
        }

        override fun removeRequirement(){
            if (!required){
                println("A requirement was removed; however, this requirement was never set in the first place.")
            }
            required = false
        }

        public inner class PoseEstimator(
            override val cameraYaw: Angle
        ): VisionPoseSupplier {
            override val robotPoseEstimate: Measurement<UnitPose2d>?
                by logInputs.nullableValue(default = Measurement(UnitPose2d(), Time(0.0))){ getPoseEstimate() }

            private fun getPoseEstimate(): Measurement<UnitPose2d>?{
                if (isSimulation() || !hasTargets() || getCurrentPipelineIndex(name).toInt() != index) {
                    return null
                }

                val allianceColor: DriverStation.Alliance =
                    DriverStation.getAlliance().orElse(defaultDriverStationIfUnavailable)

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

                return Measurement(
                    value = UnitPose2d(
                        poseArray[0].ofUnit(meters),
                        poseArray[1].ofUnit(meters),
                        poseArray[5].ofUnit(degrees)
                    ),
                    timestamp = fpgaTimestamp() - poseArray[6].ofUnit(milli.seconds)
                )
            }

        }
    }

    public inner class MLDetectorPipeline(
        override val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): MLClassifierPipeline(index,logInputs), VisionPipeline<VisionTarget.ML> {

        override val visionData: VisionData<VisionTarget.ML>?
            by logInputs.nullableValue(default = emptyMLVisionData()){ getMLData() }

        private fun getMLData(): VisionData<VisionTarget.ML>?{
            if (isSimulation() || !hasTargets()) {
                return null
            }else if (getCurrentPipelineIndex(name).toInt() != index){
                println("The current pipeline index for the limelight is incorrect. You must call reset() on the pipeline.")
                return null
            }

            if (useJsonDump){
                val allTargets = jsonResults.targets_Detector.map{
                    VisionTarget.ML(
                        tx = it.tx,
                        ty = it.ty,
                        areaPercent = it.ta,
                        id = it.classID.toInt()
                    )
                }.toMutableList()
                val bestTarget = allTargets.removeAt(0)
                val latency = jsonResults.latency_capture.ofUnit(milli.seconds)

                return VisionData(fpgaTimestamp() - latency, bestTarget, allTargets)
            }else{
                val latency = (getLatency_Capture(name) + getLatency_Pipeline(name)).ofUnit(milli.seconds)

                return VisionData(
                    fpgaTimestamp() - latency,
                    VisionTarget.ML(
                        getTX(name),
                        getTY(name),
                        getTA(name),
                        getNeuralClassID(name).toInt()
                    )
                )
            }
        }

        override val lensHeight: Distance = this@ChargerLimelight.lensHeight

        override val mountAngle: Angle = this@ChargerLimelight.mountAngle
    }

    public open inner class MLClassifierPipeline(
        private val index: Int,
        /**
         * The namespace of which the Limelight Pipeline logs to:
         * Ensure that this namespace is the same across real and sim equivalents.
         * @see LoggableInputsProvider
         */
        logInputs: LoggableInputsProvider
    ): Classifier<Int?> {

        init{
            if (index < 0 || index > 9){
                error("Your pipeline's ID is out of range.")
            }
            reset()
        }

        final override fun reset(){
            setPipelineIndex(name,index)
            println("Limelight with name $name has had it's pipeline reset to $index")
        }

        final override val itemType: Int?
            by logInputs.nullableInt{
                if (getCurrentPipelineIndex(name).toInt() == index && hasTargets() && isReal()){
                    if (useJsonDump){
                        jsonResults.targets_Classifier[0].classID.toInt()
                    }else{
                        getNeuralClassID(name).toInt()
                    }
                }else{
                    if (getCurrentPipelineIndex(name).toInt() != index){
                        println("The current pipeline index for the limelight of name '$name' is incorrect. You must call reset() on the pipeline.")
                    }
                    null
                }
            }

        final override fun require(){
            if (required){
                error("A Limelight with name '$name' has been required in 2 different places. \n " +
                        "Make sure to call pipeline.isRequired = false at the end of all commands!"
                )
            }
            required = true
        }

        final override fun removeRequirement(){
            if (!required){
                println("A requirement was removed; however, this requirement was never set in the first place.")
            }
            required = false
        }
    }
}
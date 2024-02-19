@file:Suppress("unused")
package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.Distance


/**
 * An apriltag pipeline that fuses together multiple different pipelines.
 */
class FusedAprilTagPipeline(
    private val shouldAverageBestTargets: Boolean = true,
    private val pipelines: List<AprilTagVisionPipeline>
): AprilTagVisionPipeline {

    constructor(
        vararg pipelines: AprilTagVisionPipeline
    ): this(
        shouldAverageBestTargets = true,
        pipelines.toList()
    )

    /*
    2 constructors are necessary due to the way that varargs interact with default variables
     */
    constructor(
        shouldAverageBestTargets: Boolean,
        vararg pipelines: AprilTagVisionPipeline
    ): this(
        shouldAverageBestTargets,
        pipelines.toList()
    )

    init{
        // ensures that all pipelines added are distinct and aren't from the same camera
        val camNames: MutableList<String> = mutableListOf()
        for (pipeline in pipelines){
            val camName = pipeline.cameraConstants.cameraName
            if (camName in camNames){
                error(
                    "You cannot add 2 vision pipelines from the same vision camera into a fused vision pipeline" +
                    "(as they consume the same camera resource). Camera name: $camName"
                )
            }else{
                camNames.add(camName)
            }
        }
    }

    /**
     * Fetches Vision target data from all pipelines, then compiles it into a single list.
     */
    override val visionTargets: List<VisionTarget.AprilTag>
        get(){
            val allData = pipelines.map { it.visionTargets }
            val resultingData = mutableListOf<VisionTarget.AprilTag>()
            for (data in allData){
                resultingData.addAll(data)
            }
            return resultingData
        }

    /**
     * Takes the best vision target from each camera, and averages them together.
     */
    override val bestTarget: VisionTarget.AprilTag?
        get(){
            if (shouldAverageBestTargets){
                val bestTargets = pipelines.mapNotNull { it.bestTarget }
                return if (bestTargets.isEmpty()){
                    null
                }else{
                    VisionTarget.AprilTag(
                        bestTargets.map{ it.timestamp }.average(),
                        bestTargets.map{ it.tx }.average(),
                        bestTargets.map{ it.ty }.average(),
                        bestTargets.map{ it.areaPercent }.average(),
                        bestTargets[0].fiducialId,
                        bestTargets[0].targetTransformFromCam,
                    )
                }
            }else{
                // if it is specified to not average together best target values,
                // search for the first valid(non-null) target, then return it.
                // else, return null.
                for (pipeline in pipelines){
                    val bestTarget = pipeline.bestTarget
                    if (bestTarget != null){
                        return bestTarget
                    }
                }
                return null
            }
        }

    override val cameraConstants: VisionCameraConstants
        get(){
            val allData = pipelines.map{ it.cameraConstants }

            return VisionCameraConstants(
                "Fused AprilTag Pipeline($allData)",
                pipelines[0].cameraConstants.robotToCameraTransform,
            )
        }

    override fun reset() {
        pipelines.forEach{
            it.reset()
        }
    }

    override fun robotToTargetDistance(targetHeight: Distance, target: VisionTarget.AprilTag?): Distance? {
        val allDistances = pipelines.mapNotNull{ it.robotToTargetDistance(targetHeight, target) }
        return if (allDistances.isEmpty()) {
            null
        } else {
            allDistances.average()
        }
    }
}



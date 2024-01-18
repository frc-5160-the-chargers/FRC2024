@file:Suppress("unused")
package frc.chargers.hardware.sensors.vision


import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.interop.sum
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance


/**
 * An apriltag pipeline that fuses together multiple different pipelines.
 */
class FusedAprilTagPipeline(
    private val pipelines: List<AprilTagVisionPipeline>
): AprilTagVisionPipeline {

    init{
        // ensures that all pipelines added are distinct and aren't from the same camera
        val classNames: MutableList<String> = mutableListOf()
        for (pipeline in pipelines){
            val className = pipeline::class.simpleName ?: continue  // continues if added class is anonymous(has no name)
            if (className in classNames){
                error("You cannot add 2 pipelines of the same type together in a Fused vision pipeline" +
                        "(as they consume the same camera resource). Class name: $className"
                )
            }else {
                classNames.add(className)
            }
        }
    }



    private fun getTargetDataAverage(targetData: List<VisionTarget.AprilTag>): VisionTarget.AprilTag {
        require(targetData.isNotEmpty()){ "Target data for getTargetDataAverage() must not be empty." }

        val tx = targetData.map{ it.tx }.average()
        val ty = targetData.map{ it.ty }.average()
        val areaPercent = targetData.map{ it.areaPercent }.average()
        val targetTransform = targetData[0].targetTransformFromCam
        val id = targetData[0].id
        return VisionTarget.AprilTag(
            tx, ty, areaPercent, id, targetTransform
        )
    }

    override val visionData: NonLoggableVisionData<VisionTarget.AprilTag>?
        get(){
            val allVisionData: List<NonLoggableVisionData<VisionTarget.AprilTag>> =
                pipelines.mapNotNull { it.visionData }

            if (allVisionData.isEmpty()) return null

            val bestTarget = getTargetDataAverage(allVisionData.map{ it.bestTarget })
            val otherTargets: MutableList<VisionTarget.AprilTag> = mutableListOf()
            for (secondaryTargets in allVisionData.map{ it.otherTargets } ){
                otherTargets.addAll(secondaryTargets)
            }

            return NonLoggableVisionData(
                // custom .sum() overload for kmeasure
                allVisionData.map{ it.timestamp }.sum(),
                bestTarget,
                otherTargets
            )
        }

    override val bestTarget: VisionTarget.AprilTag?
        get(){
            val bestTargets = pipelines.mapNotNull { it.bestTarget }
            return if (bestTargets.isEmpty()){
                null
            }else{
                getTargetDataAverage(bestTargets)
            }
        }

    override val lensHeight: Distance
        get() = pipelines.map{ it.lensHeight }.average() // custom .average() overload for kmeasure
    override val mountAngle: Angle
        get() = pipelines.map{ it.mountAngle }.average() // custom .average() overload for kmeasure

    override fun reset() {
        pipelines.forEach{
            it.reset()
        }
    }

    override fun require() {
        pipelines.forEach {
            it.require()
        }
    }

    override fun removeRequirement() {
        pipelines.forEach{
            it.removeRequirement()
        }
    }

    override fun distanceToTarget(targetHeight: Distance, target: VisionTarget.AprilTag?): Distance? {
        val allDistances = pipelines.mapNotNull{ it.distanceToTarget(targetHeight, target) }
        return if (allDistances.isEmpty()) {
            null
        } else {
            allDistances.average()
        }
    }
}



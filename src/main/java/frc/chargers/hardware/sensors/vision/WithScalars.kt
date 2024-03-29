@file:Suppress("unused")
package frc.chargers.hardware.sensors.vision


/**
 * Adds scalars onto the tx, ty and areaPercent terms of an [AprilTagVisionPipeline].
 */
@JvmName("withScalarsAprilTag")
fun AprilTagVisionPipeline.withScalars(
    txScalar: Double = 1.0,
    tyScalar: Double = 1.0,
    areaScalar: Double = 1.0
) = object: AprilTagVisionPipeline by this{ // provides default implementations

    fun VisionTarget.AprilTag.withScalar(): VisionTarget.AprilTag =
        VisionTarget.AprilTag(
            timestamp,
            tx * txScalar,
            ty * tyScalar,
            areaPercent * areaScalar,
            fiducialId,
            targetTransformFromCam
        )

    override val visionTargets: List<VisionTarget.AprilTag>
        get() = this@withScalars.visionTargets.map{ it.withScalar() }

    override val bestTarget: VisionTarget.AprilTag?
        get() = this@withScalars.bestTarget?.withScalar() // returns null if bestTarget is null
}

/**
 * Adds scalars onto the tx, ty and areaPercent terms of an [AprilTagVisionPipeline].
 */
@JvmName("withScalarsObject")
fun ObjectVisionPipeline.withScalars(
    txScalar: Double = 1.0,
    tyScalar: Double = 1.0,
    areaScalar: Double = 1.0
) = object: ObjectVisionPipeline by this{ // provides default implementations

    fun VisionTarget.Object.withScalar(): VisionTarget.Object =
        VisionTarget.Object(
            timestamp,
            tx * txScalar,
            ty * tyScalar,
            areaPercent * areaScalar,
            classId
        )

    override val visionTargets: List<VisionTarget.Object>
        get() = this@withScalars.visionTargets.map{ it.withScalar() }

    override val bestTarget: VisionTarget.Object?
        get() = this@withScalars.bestTarget?.withScalar() // returns null if bestTarget is null
}

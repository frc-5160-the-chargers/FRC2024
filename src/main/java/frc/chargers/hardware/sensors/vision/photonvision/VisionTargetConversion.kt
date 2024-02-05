package frc.chargers.hardware.sensors.vision.photonvision

import com.batterystaple.kmeasure.quantities.Time
import frc.chargers.hardware.sensors.vision.VisionTarget
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import org.photonvision.targeting.PhotonTrackedTarget

fun toAprilTagTarget(target: PhotonTrackedTarget, timestamp: Time) =
    VisionTarget.AprilTag(
        timestamp,
        tx = target.yaw,
        ty = target.pitch,
        areaPercent = target.area,
        fiducialId = target.fiducialId,
        targetTransformFromCam = UnitTransform3d(target.bestCameraToTarget)
    )

fun toObjectTarget(target: PhotonTrackedTarget, timestamp: Time) =
    VisionTarget.Object(
        timestamp,
        tx = target.yaw,
        ty = target.pitch,
        areaPercent = target.area,
        classId = null // photonvision does not support class id's so far.
    )
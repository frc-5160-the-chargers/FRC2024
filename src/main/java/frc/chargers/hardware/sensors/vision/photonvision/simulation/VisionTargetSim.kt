@file:Suppress("unused")
package frc.chargers.hardware.sensors.vision.photonvision.simulation

import com.batterystaple.kmeasure.units.meters
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.VisionTargetSim

fun VisionTargetSim(
    pose: UnitPose3d,
    targetModel: TargetModel,
    id: Int? = null,
): VisionTargetSim = if (id == null){
    VisionTargetSim(pose.inUnit(meters), targetModel)
}else{
    VisionTargetSim(pose.inUnit(meters), targetModel, id)
}



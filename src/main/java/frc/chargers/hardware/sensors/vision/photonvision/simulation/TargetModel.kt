@file:Suppress("unused")
package frc.chargers.hardware.sensors.vision.photonvision.simulation

import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTranslation3d
import org.photonvision.estimation.TargetModel

fun TargetModel(
    width: Length,
    height: Length
): TargetModel =
    TargetModel(width.inUnit(meters), height.inUnit(meters))

fun TargetModel(
    diameter: Length
): TargetModel = TargetModel(diameter.inUnit(meters))

fun TargetModel(
    length: Length,
    width: Length,
    height: Length
): TargetModel =
    TargetModel(length.inUnit(meters), width.inUnit(meters), height.inUnit(meters))


fun TargetModel(
    vararg vertices: UnitTranslation3d
): TargetModel =
    TargetModel(vertices.map{ it.inUnit(meters) })
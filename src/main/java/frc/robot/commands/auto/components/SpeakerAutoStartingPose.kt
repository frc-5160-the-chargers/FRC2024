@file:Suppress("unused")
package frc.robot.commands.auto.components

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d

enum class SpeakerAutoStartingPose(val pose: UnitPose2d){
    LEFT(UnitPose2d()),
    CENTER(UnitPose2d(1.45.meters, 5.546.meters, 180.degrees)),
    RIGHT(UnitPose2d())
}
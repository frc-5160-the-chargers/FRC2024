package frc.robot.commands.auto.components

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d

object AutoStartingPose {
    val AMP_BLUE = UnitPose2d(1.38.meters, 7.29.meters, -90.degrees)

    val SPEAKER_CENTER_BLUE = UnitPose2d(1.415.meters, 5.573.meters, 0.degrees)

    @Suppress("unused")
    val SPEAKER_RIGHT_BLUE = UnitPose2d(1.29.meters, 4.66.meters, -0.785.radians)
}
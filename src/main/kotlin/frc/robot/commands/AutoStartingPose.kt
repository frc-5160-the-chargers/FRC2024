package frc.robot.commands

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import frc.chargers.wpilibextensions.Pose2d

@Suppress("unused")
object AutoStartingPose {
    val AMP_BLUE = Pose2d(1.38.meters, 7.29.meters, -90.degrees)

    val SPEAKER_CENTER_BLUE = Pose2d(1.415.meters, 5.573.meters, 0.degrees)

    val SPEAKER_RIGHT_BLUE = Pose2d(1.29.meters, 4.66.meters, -0.785.radians)

    val SPEAKER_LEFT_BLUE = Pose2d(0.768.meters, 6.689.meters, 1.074.radians)
}
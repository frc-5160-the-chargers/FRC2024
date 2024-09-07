package frc.robot.rigatoni

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import frc.chargers.wpilibextensions.Pose2d
import frc.chargers.wpilibextensions.flipWhenRedAlliance

object AutoStartingPose {
    fun getAmp() = Pose2d(1.38.meters, 7.29.meters, -90.degrees).flipWhenRedAlliance()

    fun getSpeakerCenter() = Pose2d(1.415.meters, 5.573.meters, 0.degrees).flipWhenRedAlliance()

    fun getSpeakerRight() = Pose2d(1.29.meters, 4.66.meters, -0.785.radians).flipWhenRedAlliance()

    fun getSpeakerLeft() = Pose2d(0.768.meters, 6.689.meters, 1.074.radians).flipWhenRedAlliance()
}



package frc.robot.hardware.subsystems.pivot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.radians


object PivotAngle {
    val AMP: Angle = 0.72.radians

    val SOURCE: Angle = 0.degrees

    val GROUND_INTAKE_HANDOFF: Angle = -0.9.radians

    val STOWED: Angle = GROUND_INTAKE_HANDOFF // same as of now

    val SPEAKER: Angle = GROUND_INTAKE_HANDOFF // same as of now

    val STARTING: Angle = -1.15.radians
}
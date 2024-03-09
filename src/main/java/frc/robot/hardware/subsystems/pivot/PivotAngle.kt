package frc.robot.hardware.subsystems.pivot

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.radians
import edu.wpi.first.wpilibj.RobotBase

object PivotAngle {
    val AMP: Angle = if (RobotBase.isReal()) 0.55.radians else 30.degrees

    val SOURCE: Angle = if (RobotBase.isReal()) 0.degrees else 0.degrees

    val GROUND_INTAKE_HANDOFF: Angle = if (RobotBase.isReal()) -1.7.radians else -70.degrees

    val STOWED: Angle = GROUND_INTAKE_HANDOFF // same as of now

    val SPEAKER: Angle = GROUND_INTAKE_HANDOFF // same as of now
}
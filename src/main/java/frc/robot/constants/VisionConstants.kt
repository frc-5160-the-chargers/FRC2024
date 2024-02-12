package frc.robot.constants

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import frc.chargers.wpilibextensions.geometry.threedimensional.Rotation3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTranslation3d

val ROBOT_TO_LIMELIGHT = UnitTransform3d(
    UnitTranslation3d(
        0.25.meters, 0.meters, 1.meters
    ),
    Rotation3d(
        0.degrees,
        -35.degrees,
        0.degrees
    )
)

val ROBOT_TO_APRILTAG_PHOTON_CAM = UnitTransform3d(
    UnitTranslation3d(
        0.25.meters, 0.meters, 1.meters
    ),
    Rotation3d(
        0.degrees,
        -25.degrees,
        0.degrees
    )
)

val ROBOT_TO_ML_PHOTON_CAM = UnitTransform3d()

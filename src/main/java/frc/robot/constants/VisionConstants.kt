package frc.robot.constants

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import frc.chargers.wpilibextensions.geometry.threedimensional.Rotation3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTranslation3d

val ROBOT_TO_LIMELIGHT = UnitTransform3d(
    UnitTranslation3d(
        0.5.meters, -0.1.meters, 0.5.meters
    ),
    Rotation3d(
        0.degrees,
        -30.degrees,
        0.degrees
    )
)

val ROBOT_TO_APRILTAG_PHOTON_CAM = UnitTransform3d(
    UnitTranslation3d(
        0.5.meters, 0.1.meters, 0.5.meters
    ),
    Rotation3d(
        0.degrees,
        -30.degrees,
        0.degrees
    )
)

val ROBOT_TO_ML_PHOTON_CAM = UnitTransform3d()

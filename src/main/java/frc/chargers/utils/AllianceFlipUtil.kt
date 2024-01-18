@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asAngle
import java.util.*

/*
Credits go to 6328 & 4099 for all of the code below;
some edits were applied to better suit kotlin(such as swapping to extension functions).
 */

private val FIELD_LENGTH = Distance(
    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().fieldLength
)

/** Flips a blue alliance translation to the correct side of the field based on the current alliance color. */
public fun UnitTranslation2d.flipWhenNeeded(): UnitTranslation2d =
    if (shouldFlip()) {
        UnitTranslation2d(FIELD_LENGTH - x, y)
    } else {
        this
    }

/** Flips an x coordinate to the correct side of the field based on the current alliance color. */
public fun flipXCoordWhenNeeded(xCoordinate: Length): Length =
    if (shouldFlip()) {
        FIELD_LENGTH - xCoordinate
    } else {
        xCoordinate
    }


/** Flips a rotation based on the current alliance color. */
public fun Angle.flipWhenNeeded(): Angle =
    if (shouldFlip()) {
        Rotation2d(-cos(this), sin(this)).asAngle()
    } else {
        this
    }

/** Flips a blue alliance pose to the correct side of the field based on the current alliance color. */
public fun UnitPose2d.flipWhenNeeded(): UnitPose2d =
    if (shouldFlip()) {
        UnitPose2d(
            FIELD_LENGTH - x, y, Rotation2d(-cos(rotation),sin(rotation)).asAngle()
        )
    } else {
        this
    }

private fun shouldFlip(): Boolean = DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)
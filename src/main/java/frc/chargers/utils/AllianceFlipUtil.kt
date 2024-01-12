@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitPose2d
import frc.chargers.wpilibextensions.geometry.twodimensional.UnitTranslation2d
import frc.chargers.wpilibextensions.geometry.twodimensional.asAngle

/*
Credits go to 6328 & 4099 for all of the code below;
some edits were applied to better suit kotlin(such as swapping to extension functions).
 */

/** Flips a translation to the correct side of the field based on the current alliance color. */
public fun UnitTranslation2d.applyFlip(fieldLength: Distance): UnitTranslation2d =
    if (shouldFlip()) {
        UnitTranslation2d(fieldLength - x, y)
    } else {
        this
    }

/** Flips an x coordinate to the correct side of the field based on the current alliance color. */
public fun flipXCoord(xCoordinate: Length, fieldLength: Distance): Length =
    if (shouldFlip()) {
        fieldLength - xCoordinate
    } else {
        xCoordinate
    }


/** Flips a rotation based on the current alliance color. */
public fun Angle.applyFlip(): Angle =
    if (shouldFlip()) {
        Rotation2d(-cos(this), sin(this)).asAngle()
    } else {
        this
    }

/** Flips a pose to the correct side of the field based on the current alliance color. */
public fun UnitPose2d.applyFlip(fieldLength: Distance): UnitPose2d =
    if (shouldFlip()) {
        UnitPose2d(
            fieldLength - x, y, Rotation2d(-cos(rotation),sin(rotation)).asAngle()
        )
    } else {
        this
    }

private fun shouldFlip(): Boolean =
    try{
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    }catch(_: Exception){
        false
    }
@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d

/*
    The following functions are used to provide dummy vision data
    To be logged,
    when the data is marked invalid.

    Since data must be logged every loop, these values are fallen back to when nessecary.
 */

public fun emptyAprilTagVisionData(): VisionData<VisionTarget.AprilTag> =
    VisionData(
        timestamp = fpgaTimestamp(),
        bestTarget = VisionTarget.AprilTag(
            0.0,0.0,0.0,0, UnitTransform3d()
        )
    )

public fun emptyMLVisionData(): VisionData<VisionTarget.ML> =
    VisionData(
        timestamp = fpgaTimestamp(),
        bestTarget = VisionTarget.ML(
            0.0,0.0,0.0,0
        )
    )

public fun emptyGenericVisionData(): VisionData<VisionTarget.Generic> =
    VisionData(
        timestamp = fpgaTimestamp(),
        bestTarget = VisionTarget.Generic(0.0,0.0,0.0)
    )
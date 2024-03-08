package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.quantities.Area
import com.batterystaple.kmeasure.quantities.Distance

interface BoundingBox2d {
    operator fun contains(other: BoundingBox2d): Boolean

    operator fun contains(other: UnitPose2d): Boolean

    operator fun plus(other: UnitTransform2d)

    val area: Area

    val perimeter: Distance
}
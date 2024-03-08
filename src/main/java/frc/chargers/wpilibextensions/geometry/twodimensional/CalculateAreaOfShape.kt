package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.quantities.Area
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.meters
import kotlin.math.abs

/**
 * A utility function that uses the shoelace theorem to calculate area.
 */
fun calculateAreaOfShape(
    vararg vertices: UnitPose2d
): Area {
    val xCoordinates = vertices.map{ it.x.inUnit(meters) }
    val yCoordinates = vertices.map{ it.y.inUnit(meters) }

    var sum = 0.0

    for (i in 0..<vertices.size-1){
        sum += xCoordinates[i] * yCoordinates[i+1]
        sum -= xCoordinates[i+1] * yCoordinates[i]
    }

    return (abs(sum) / 2.0).ofUnit(meters * meters)
}
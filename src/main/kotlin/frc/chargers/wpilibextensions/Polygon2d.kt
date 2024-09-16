@file:Suppress("unused")
package frc.chargers.wpilibextensions

import edu.wpi.first.math.geometry.Translation2d

/**
 * A class that represents a polygon in 2d space.
 *
 * Credits: FRC 3173(IgKnighters)
 */
data class Polygon2d(val vertices: List<Translation2d>) {
    constructor(vararg vertices: Translation2d) : this(vertices.toList())

    operator fun contains(point: Translation2d): Boolean {
        var j: Int
        var c = false
        var i = 0
        j = vertices.size - 1
        while (i < vertices.size) {
            if (((vertices[i].y > point.y) != (vertices[j].y > point.y))
                && (point.x < (vertices[j].x - vertices[i].x)
                        * (point.y - vertices[i].y) / (vertices[j].y - vertices[i].y)
                        + vertices[i].x)
            ) {
                c = !c
            }
            j = i++
        }
        return c
    }

    operator fun contains(polygon: Polygon2d): Boolean =
        polygon.vertices.all{ contains(it) }

    fun intersects(polygon: Polygon2d): Boolean =
        (vertices + polygon.vertices).any{ contains(it) }
}
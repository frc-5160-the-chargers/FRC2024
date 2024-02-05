@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.hypot
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Transform2d
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.wpilibextensions.geometry.ofUnit
import org.littletonrobotics.junction.LogTable

/**
 * A wrapper for WPILib's [Transform2d], adding in Unit support.
 */
public data class UnitTransform2d(
    public val siValue: Transform2d = Transform2d()
): AdvantageKitLoggable<UnitTransform2d>{


    public constructor(translation: UnitTranslation2d, rotation: Angle): this(
        Transform2d(translation.siValue,rotation.asRotation2d())
    )

    public constructor(initial: UnitPose2d, last: UnitPose2d): this(
        Transform2d(initial.siValue,last.siValue)
    )

    public constructor(x: Distance, y: Distance, rotation: Angle): this(
        Transform2d(UnitTranslation2d(x,y).siValue, rotation.asRotation2d())
    )

    /**
     * The translation component of the transform.
     *
     * @see Transform2d.getTranslation
     */
    public val translation: UnitTranslation2d get() = siValue.translation.ofUnit(meters)

    /**
     * The rotation component of the transform.
     *
     * @see Transform2d.getRotation
     */
    public val rotation: Angle get() = siValue.rotation.asAngle()

    /**
     * The X distance component of the transform.
     *
     * @see Transform2d.getX
     */
    public val x: Distance get() = siValue.x.meters

    /**
     * The Y distance component of the transform.
     *
     * @see Transform2d.getY
     */
    public val y: Distance get() = siValue.y.meters


    public val norm: Distance get() = hypot(x, y)


    /**
     * Converts this object to a [Transform2d] with specified distance units.
     */
    public fun inUnit(unit: Distance): Transform2d = Transform2d(
        translation.inUnit(unit),
        rotation.asRotation2d()
    )



    public operator fun div(scalar: Double): UnitTransform2d = UnitTransform2d(siValue / scalar)
    public operator fun times(scalar: Double): UnitTransform2d = UnitTransform2d(siValue * scalar)
    public operator fun plus(other: UnitTransform2d): UnitTransform2d = UnitTransform2d(siValue + other.siValue)


    public operator fun unaryMinus(): UnitTransform2d = UnitTransform2d(siValue.inverse())
    override fun pushToLog(table: LogTable, category: String) {
        table.put(category, Transform2d.struct, siValue)
    }

    override fun getFromLog(table: LogTable, category: String): UnitTransform2d =
        UnitTransform2d(table.get(category, Transform2d.struct, Transform2d()))


}
@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.Interpolatable
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.wpilibextensions.geometry.ofUnit
import org.littletonrobotics.junction.LogTable

// A function must be used here, as a second constructor would cause a platform declaration crash.
public fun UnitTranslation2d(norm: Distance, angle: Angle): UnitTranslation2d =
    UnitTranslation2d(
        Translation2d(norm.siValue,angle.asRotation2d())
    )

/**
 * A wrapper for WPILib's [Translation2d], adding in Unit support.
 */
public data class UnitTranslation2d(
    public val siValue: Translation2d = Translation2d()
): Interpolatable<UnitTranslation2d>, AdvantageKitLoggable<UnitTranslation2d> {

    public constructor(x: Distance, y: Distance): this(Translation2d(x.siValue,y.siValue))


    /**
     * The X distance component of the Translation.
     *
     * @see Translation2d.getX
     */
    public val x: Distance get() = Distance(siValue.x)

    /**
     * The Y distance component of the Translation.
     *
     * @see Translation2d.getY
     */
    public val y: Distance get() = Distance(siValue.y)

    /**
     * The distance this translation is from the origin.
     *
     * @see Translation2d.getNorm
     */
    public val norm: Distance get() = Distance(siValue.norm)


    public fun inUnit(unit: Distance): Translation2d = Translation2d(x.inUnit(unit),y.inUnit(unit))
    public operator fun minus(other: UnitTranslation2d): UnitTranslation2d = siValue.minus(other.inUnit(meters)).ofUnit(meters)
    public operator fun div(scalar: Double): UnitTranslation2d = (siValue/scalar).ofUnit(meters)
    public operator fun times(scalar: Double): UnitTranslation2d = (siValue*scalar).ofUnit(meters)
    public operator fun plus(other: UnitTranslation2d): UnitTranslation2d = (siValue + other.inUnit(meters)).ofUnit(meters)
    public operator fun unaryMinus(): UnitTranslation2d = (-siValue).ofUnit(meters)


    public fun getDistance(other: UnitTranslation2d): Distance = siValue.getDistance(other.inUnit(meters)).meters
    override fun interpolate(other: UnitTranslation2d, t: Double): UnitTranslation2d = siValue.interpolate(other.inUnit(meters),t).ofUnit(meters)
    public fun rotateBy(other: Angle): UnitTranslation2d = siValue.rotateBy(other.asRotation2d()).ofUnit(meters)
    override fun pushToLog(table: LogTable, category: String) {
        table.put(category, Translation2d.struct, siValue)
    }
    override fun getFromLog(table: LogTable, category: String): UnitTranslation2d =
        UnitTranslation2d(table.get(category, Translation2d.struct, Translation2d()))

}


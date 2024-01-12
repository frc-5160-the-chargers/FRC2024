@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.geometry.threedimensional

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.inUnit
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.Interpolatable
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import org.littletonrobotics.junction.LogTable


public data class UnitTranslation3d(
    public val siValue: Translation3d = Translation3d()
): Interpolatable<UnitTranslation3d>, AdvantageKitLoggable<UnitTranslation3d> {

    public constructor(x: Distance, y: Distance, z: Distance): this(
        siValue = Translation3d(x.siValue,y.siValue,z.siValue)
    )

    public constructor(distance: Distance, rotation: Rotation3d): this(
        siValue = Translation3d(distance.siValue,rotation)
    )

    public val norm: Distance get() = Distance(siValue.norm)

    public fun getDistance(other: UnitTranslation3d): Distance = Distance(siValue.getDistance(other.siValue))

    public val x: Distance get() = Distance(siValue.x)
    public val y: Distance get() = Distance(siValue.y)
    public val z: Distance get() = Distance(siValue.z)


    public fun inUnit(unit: Distance): Translation3d = Translation3d(
        x.inUnit(unit),
        y.inUnit(unit),
        z.inUnit(unit)
    )

    public operator fun div(scalar: Double): UnitTranslation3d = UnitTranslation3d(siValue/scalar)
    public operator fun times(scalar: Double): UnitTranslation3d = UnitTranslation3d(siValue * scalar)
    public operator fun plus(other: UnitTranslation3d): UnitTranslation3d = UnitTranslation3d(siValue + other.siValue)
    public operator fun minus(other: UnitTranslation3d): UnitTranslation3d = UnitTranslation3d(siValue - other.siValue)

    public operator fun unaryMinus(): UnitTranslation3d = UnitTranslation3d(-siValue)
    override fun interpolate(endValue: UnitTranslation3d, t: Double): UnitTranslation3d =
        UnitTranslation3d(siValue.interpolate(endValue.siValue,t))

    override fun pushToLog(table: LogTable, category: String) {
        table.put(category, Translation3d.struct, siValue)
    }

    override fun getFromLog(table: LogTable, category: String): UnitTranslation3d =
        UnitTranslation3d(table.get(category, Translation3d.struct, Translation3d()))

}


@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.geometry.threedimensional

import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.quantities.Distance
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.utils.math.units.KmeasureUnit
import org.littletonrobotics.junction.LogTable


public data class UnitTransform3d(
    public val siValue: Transform3d = Transform3d()
): AdvantageKitLoggable<UnitTransform3d> {

    public constructor(translation: UnitTranslation3d, rotation: Rotation3d): this(
        Transform3d(translation.siValue,rotation)
    )

    public fun inUnit(unit: KmeasureUnit<DistanceDimension>): Transform3d = Transform3d(
        translation.inUnit(unit),
        rotation
    )

    public val translation: UnitTranslation3d get() = UnitTranslation3d(siValue.translation)

    public val rotation: Rotation3d get() = siValue.rotation

    public val x: Distance get() = Distance(siValue.x)
    public val y: Distance get() = Distance(siValue.y)
    public val z: Distance get() = Distance(siValue.z)


    public operator fun div(scalar: Double): UnitTransform3d = UnitTransform3d(siValue / scalar)
    public operator fun times(scalar: Double): UnitTransform3d = UnitTransform3d(siValue * scalar)
    public operator fun plus(other: UnitTransform3d): UnitTransform3d = UnitTransform3d(siValue + other.siValue)

    override fun pushToLog(table: LogTable, category: String) {
        table.put(category, Transform3d.struct, siValue)
    }

    override fun getFromLog(table: LogTable, category: String): UnitTransform3d =
        UnitTransform3d(table.get(category,Transform3d.struct, Transform3d()))

}
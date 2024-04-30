@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.wpilibextensions.geometry.twodimensional

import com.batterystaple.kmeasure.dimensions.DistanceDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.interpolation.Interpolatable
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import frc.chargers.utils.encoding.createCopy
import frc.chargers.utils.math.units.KmeasureUnit
import frc.chargers.wpilibextensions.geometry.ofUnit
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d

/**
 * A wrapper around WPILib's [UnitPose2d] class, adding in units support.
 */
public data class UnitPose2d(
    val siValue: Pose2d = Pose2d()
): Interpolatable<UnitPose2d>, StructSerializable {
    companion object{
        @JvmStatic
        val struct: Struct<UnitPose3d> = createCopy(
            Pose3d.struct,
            convertor = { it.ofUnit(meters) },
            inverseConvertor = { it.inUnit(meters) }
        )
    }

    public constructor(translation: UnitTranslation2d, rotation: Angle = Angle(0.0)): this(
        Pose2d(translation.siValue, rotation.asRotation2d())
    )

    public constructor(x: Distance, y: Distance, rotation: Angle = Angle(0.0)): this(
        Pose2d(x.siValue,y.siValue,rotation.asRotation2d())
    )

    /**
     * The translation component of the pose.
     *
     * @see Pose2d.getTranslation
     */
    public val translation: UnitTranslation2d get() = UnitTranslation2d(siValue.translation)

    /**
     * The rotation component of the pose.
     *
     * @see Pose2d.getRotation
     */
    public val rotation: Angle get() = siValue.rotation.asAngle()

    /**
     * The X distance component of the pose.
     *
     * @see Pose2d.getX
     */
    public val x: Distance get() = Distance(siValue.x)

    /**
     * The y component of the pose.
     *
     * @see Pose2d.getY
     */
    public val y: Distance get() = Distance(siValue.y)

    /**
     * Converts this pose to a [UnitPose3d].
     */
    public fun toPose3d(): UnitPose3d = UnitPose3d(Pose3d(siValue))

    /**
     * Fetches the distance from this pose to another pose.
     */
    public fun distanceTo(other: UnitPose2d): Distance =
        abs((this - other).norm)


    /**
     * Converts this pose to a pose with a specific specified unit.
     */
    public fun inUnit(unit: KmeasureUnit<DistanceDimension>): Pose2d =
        Pose2d(translation.inUnit(unit),rotation.asRotation2d())

    public operator fun plus(other: UnitTransform2d): UnitPose2d = UnitPose2d(siValue + other.siValue)
    public operator fun minus(other: UnitPose2d): UnitTransform2d = UnitTransform2d(siValue - other.siValue)
    public operator fun times(scalar: Double): UnitPose2d = UnitPose2d(siValue * scalar)
    public operator fun div(scalar: Double): UnitPose2d = UnitPose2d(siValue / scalar)

    override fun interpolate(endValue: UnitPose2d, t: Double): UnitPose2d =
        UnitPose2d(siValue.interpolate(endValue.siValue, t))


    /**
     * @see Pose2d.nearest
     */
    fun nearest(otherPoses: List<UnitPose2d>): UnitPose2d =
        UnitPose2d(siValue.nearest(otherPoses.map{ it.siValue }))
}
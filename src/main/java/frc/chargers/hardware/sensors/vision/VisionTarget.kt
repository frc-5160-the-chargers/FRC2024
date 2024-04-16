@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.util.struct.Struct
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import java.nio.ByteBuffer

/**
 * Represents a Vision Result, which contains all the data that is applicable to a single vision target.
 */
public sealed interface VisionTarget{

    public val timestamp: Time
    public val tx: Double
    public val ty: Double
    public val areaPercent: Double


    public open class Object(
        override val timestamp: Time,
        override val tx: Double,
        override val ty: Double,
        override val areaPercent: Double,
        open val classId: String? = null
    ): VisionTarget {
        companion object{
            val struct = object: Struct<Object> {
                override fun getTypeClass(): Class<Object> = Object::class.java

                override fun getTypeString(): String {
                    TODO("Not yet implemented")
                }

                override fun getSize(): Int {
                    TODO("Not yet implemented")
                }

                override fun getSchema(): String {
                    TODO("Not yet implemented")
                }

                override fun unpack(p0: ByteBuffer?): Object {
                    TODO("Not yet implemented")
                }

                override fun pack(p0: ByteBuffer?, p1: Object?) {
                    TODO("Not yet implemented")
                }
            }
        }
    }

    public open class AprilTag(
        override val timestamp: Time,
        override val tx: Double,
        override val ty: Double,
        override val areaPercent: Double,
        public val fiducialId: Int,
        public val targetTransformFromCam: UnitTransform3d
    ): VisionTarget {

    }
}
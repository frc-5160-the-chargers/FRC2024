package frc.chargers.hardware.sensors.visionredo

import edu.wpi.first.util.struct.Struct
import java.nio.ByteBuffer

data class VisionTarget(
    val tx: Double,
    val ty: Double,
    val areaPercent: Double,
    val type: Type
){
    companion object{
        val struct = object: Struct<VisionTarget>{
            override fun getTypeClass(): Class<VisionTarget> = VisionTarget::class.java

            override fun getTypeString(): String = "struct:VisionTarget"

            override fun getSize(): Int = Struct.kSizeDouble * 3 + Struct.kSizeInt8

            override fun getSchema(): String = "double tx; double ty; double areaPercent; int id(fiducial or class);"

            override fun unpack(p0: ByteBuffer?): VisionTarget {
                TODO("Not yet implemented")
            }

            override fun pack(p0: ByteBuffer?, p1: VisionTarget?) {
                TODO("Not yet implemented")
            }

        }
    }


    sealed class Type{
        class AprilTag(val fiducialId: Int): Type()

        class Object(val classId: Any? = null): Type()
    }
}
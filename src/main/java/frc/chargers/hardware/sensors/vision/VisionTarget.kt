@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import org.littletonrobotics.junction.LogTable

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
    ): VisionTarget, AdvantageKitLoggable<Object> {

        /**
         * Represents dummy vision data for a [VisionTarget.Object]
         */
        object Dummy: Object(Time(0.0), 0.0, 0.0, 0.0, null)


        override fun pushToLog(table: LogTable, category: String) {
            table.apply{
                put("$category/timestamp", this@Object.timestamp.inUnit(seconds))
                put("$category/tx", tx)
                put("$category/ty", ty)
                put("$category/areaPercent", areaPercent)
                put("$category/hasClassId", classId != null)
                put("$category/classId", classId ?: "NULL")
            }
        }

        override fun getFromLog(table: LogTable, category: String): Object =
            Object(
                table.get("$category/timestamp", 0.0).ofUnit(seconds),
                table.get("$category/tx",0.0),
                table.get("$category/ty",0.0),
                table.get("$category/areaPercent",0.0),
                if (table.get("$category/hasClassId", false)){
                    table.get("$category/classId", classId)
                }else{
                    null
                }
            )
    }

    public open class AprilTag(
        override val timestamp: Time,
        override val tx: Double,
        override val ty: Double,
        override val areaPercent: Double,
        public val fiducialId: Int,
        public val targetTransformFromCam: UnitTransform3d
    ): VisionTarget, AdvantageKitLoggable<AprilTag> {

        /**
         * Represents dummy vision data for a [VisionTarget.AprilTag]
         */
        object Dummy: AprilTag(Time(0.0), 0.0, 0.0, 0.0, 0, UnitTransform3d())


        override fun pushToLog(table: LogTable, category: String) {
            table.apply{
                put("$category/timestamp", this@AprilTag.timestamp.inUnit(seconds))
                put("$category/tx", tx)
                put("$category/ty",ty)
                put("$category/areaPercent",areaPercent)
                put("$category/id", fiducialId)
            }
            targetTransformFromCam.pushToLog(table,"$category/transformFromCam")
        }

        override fun getFromLog(table: LogTable, category: String): AprilTag {
            return AprilTag(
                table.get("$category/timestamp", 0.0).ofUnit(seconds),
                table.get("$category/tx",0.0),
                table.get("$category/ty",0.0),
                table.get("$category/areaPercent",0.0),
                table.get("$category/id",0),
                targetTransformFromCam.getFromLog(table,"$category/transformFromCam")
            )
        }
    }
}
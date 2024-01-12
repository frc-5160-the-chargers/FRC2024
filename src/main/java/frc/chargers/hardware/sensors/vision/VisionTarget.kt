@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.hardware.sensors.vision

import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import frc.chargers.wpilibextensions.fpgaTimestamp
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import org.littletonrobotics.junction.LogTable


/**
 * Represents Vision Data(timestamp, best target, and other targets)
 *
 * that can be pushed to log.
 */
public class VisionData<out R>(
    timestamp: Time,
    bestTarget: R,
    otherTargets: List<R>
): NonLoggableVisionData<R>(timestamp, bestTarget, otherTargets), AdvantageKitLoggable<VisionData<R>>
    where R : VisionTarget, R: AdvantageKitLoggable<R>{

    public constructor(
        timestamp: Time,
        bestTarget: R,
        vararg otherTargets: R
    ): this(
        timestamp, bestTarget, listOf(*otherTargets)
    )


    override fun pushToLog(table: LogTable, category: String) {
        table.put("$category/timestampSecs", timestamp.inUnit(seconds))
        bestTarget.pushToLog(table,"$category/bestTarget")
        otherTargets.forEachIndexed{ index, target ->
            target.pushToLog(table,"$category/otherTarget$index")
        }
    }

    override fun getFromLog(table: LogTable, category: String): VisionData<R> = VisionData(
        timestamp = table.get("timestampSecs", fpgaTimestamp().inUnit(seconds)).ofUnit(seconds),
        bestTarget = bestTarget.getFromLog(table,"$category/bestTarget"),
        otherTargets = otherTargets.mapIndexed{ index, target ->
            target.getFromLog(table,"$category/otherTarget$index")
        }
    )

}

/**
 * Represents Vision Data without logging functionality.
 * Contains data about the timestamp, best target, and other targets fetched.
 */
public open class NonLoggableVisionData<out R: VisionTarget>(
    public val timestamp: Time,
    public val bestTarget: R,
    public val otherTargets: List<R>
){
    public constructor(
        timestamp: Time,
        bestTarget: R,
        vararg otherTargets: R
    ): this(timestamp, bestTarget, listOf(*otherTargets))

    public val allTargets: List<R> = otherTargets + listOf(bestTarget)

}

/**
 * Represents a Vision Result, which contains all the data that is applicable to a single vision target.
 */
public sealed interface VisionTarget{

    public val tx: Double
    public val ty: Double
    public val areaPercent: Double

    public class Generic(
        override val tx: Double,
        override val ty: Double,
        override val areaPercent: Double
    ): VisionTarget, AdvantageKitLoggable<Generic>{
        override fun pushToLog(table: LogTable, category: String) {
            table.apply{
                put("$category/tx", tx)
                put("$category/ty",ty)
                put("$category/areaPercent",areaPercent)
            }
        }

        override fun getFromLog(table: LogTable, category: String): Generic {
            return Generic(
                table.get("$category/tx",0.0),
                table.get("$category/ty",0.0),
                table.get("$category/areaPercent",0.0)
            )
        }

    }


    public class AprilTag(
        override val tx: Double,
        override val ty: Double,
        override val areaPercent: Double,
        public val id: Int,
        public val targetTransformFromCam: UnitTransform3d
    ): VisionTarget, AdvantageKitLoggable<AprilTag>{
        override fun pushToLog(table: LogTable, category: String) {
            table.apply{
                put("$category/tx", tx)
                put("$category/ty",ty)
                put("$category/areaPercent",areaPercent)
                put("$category/id",id.toLong())
            }
            targetTransformFromCam.pushToLog(table,"$category/transformFromCam")
        }

        override fun getFromLog(table: LogTable, category: String): AprilTag {
            return AprilTag(
                table.get("$category/tx",0.0),
                table.get("$category/ty",0.0),
                table.get("$category/areaPercent",0.0),
                table.get("$category/id",0),
                targetTransformFromCam.getFromLog(table,"$category/transformFromCam")
            )
        }
    }

    public class ML(
        override val tx: Double,
        override val ty: Double,
        override val areaPercent: Double,
        public val id: Int
    ): VisionTarget, AdvantageKitLoggable<ML>{
        override fun pushToLog(table: LogTable, category: String) {
            table.apply{
                put("$category/tx", tx)
                put("$category/ty",ty)
                put("$category/areaPercent",areaPercent)
                put("$category/id",id.toLong())
            }
        }

        override fun getFromLog(table: LogTable, category: String): ML {
            return ML(
                table.get("$category/tx",0.0),
                table.get("$category/ty",0.0),
                table.get("$category/areaPercent",0.0),
                table.get("$category/id",0)
            )
        }
    }

}
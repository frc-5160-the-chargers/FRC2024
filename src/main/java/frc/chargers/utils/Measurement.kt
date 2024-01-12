@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.advantagekitextensions.AdvantageKitLoggable
import org.littletonrobotics.junction.LogTable


/**
 * A [Double] accompanied by a timestamp.
 *
 * @see BasicMeasurement
 * @see QuantityMeasurement
 * @see Measurement
 */
public data class DoubleMeasurement(
    val value: Double,
    val timestamp: Time,
): AdvantageKitLoggable<DoubleMeasurement>{
    override fun pushToLog(table: LogTable, category: String) {
        table.put("$category/value", value)
        table.put("$category/timestampSecs",timestamp.inUnit(seconds))
    }

    override fun getFromLog(table: LogTable, category: String): DoubleMeasurement =
        DoubleMeasurement(
            table.get("$category/value", 0.0),
            timestamp = table.get("$category/timestampSecs",0.0).ofUnit(seconds)
        )
}

/**
 * A [Quantity] accompanied by a timestamp.
 *
 * @see BasicMeasurement
 * @see Measurement
 * @see DoubleMeasurement
 */
public data class QuantityMeasurement<D: Dimension<*,*,*,*>>(
    val value: Quantity<D>,
    val timestamp: Time,
): AdvantageKitLoggable<QuantityMeasurement<D>>{
    override fun pushToLog(table: LogTable, category: String) {
        table.put("$category/value(SI unit)", value.siValue)
        table.put("$category/timestampSecs",timestamp.inUnit(seconds))
    }

    override fun getFromLog(table: LogTable, category: String): QuantityMeasurement<D> =
        QuantityMeasurement(
            Quantity(table.get("$category/value(SI unit)", 0.0)),
            timestamp = table.get("$category/timestampSecs",0.0).ofUnit(seconds)
        )
}


/**
 * Represents a Loggable value accompanied by a timestamp.
 *
 * @see BasicMeasurement
 * @see QuantityMeasurement
 * @see DoubleMeasurement
 */
public data class Measurement<T: AdvantageKitLoggable<T>>(
    override val value: T,
    override val timestamp: Time
): BasicMeasurement<T>(value, timestamp), AdvantageKitLoggable<Measurement<T>> {
    override fun pushToLog(table: LogTable, category: String) {
        value.pushToLog(table, "$category/value")
        table.put("$category/timestampSecs",timestamp.inUnit(seconds))
    }
    override fun getFromLog(table: LogTable, category: String): Measurement<T> =
        Measurement(
            value.getFromLog(table,"$category/value"),
            timestamp = table.get("$category/timestampSecs",0.0).ofUnit(seconds)
        )
}

/**
 * Represents a simple value accompanied by a timestamp,
 * without any logging capabilities.
 *
 * @see Measurement
 * @see DoubleMeasurement
 * @see QuantityMeasurement
 */
public open class BasicMeasurement<T>(
    public open val value: T,
    public open val timestamp: Time
){
    override fun equals(other: Any?): Boolean {
        return other is BasicMeasurement<*> && value == other.value && timestamp == other.timestamp
    }

    override fun hashCode(): Int {
        var result = value?.hashCode() ?: 0
        result = 31 * result + timestamp.hashCode()
        return result
    }

}

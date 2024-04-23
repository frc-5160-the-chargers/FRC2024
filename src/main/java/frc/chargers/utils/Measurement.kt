@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.util.struct.Struct
import java.nio.ByteBuffer


/**
 * Represents a simple value accompanied by a timestamp
 */
public data class Measurement<T>(
    val value: T,
    val timestamp: Time
){
    companion object {
        fun doubleStruct(): Struct<Measurement<Double>> =
            object: Struct<Measurement<Double>> {
                private val dummyMeasurement = Measurement(0.0, Time(0.0))

                override fun getTypeClass(): Class<Measurement<Double>> = dummyMeasurement.javaClass

                override fun getTypeString(): String = "struct:DoubleMeasurement"

                override fun getSize(): Int = Struct.kSizeDouble * 2

                override fun getSchema(): String = "double value; double timestampSeconds;"

                override fun unpack(bb: ByteBuffer): Measurement<Double> =
                    Measurement(
                        bb.getDouble(),
                        bb.getDouble().ofUnit(seconds)
                    )

                override fun pack(bb: ByteBuffer, measurement: Measurement<Double>) {
                    bb.putDouble(measurement.value)
                    bb.putDouble(measurement.timestamp.inUnit(seconds))
                }
            }

        fun <D: AnyDimension> quantityStruct(): Struct<Measurement<Quantity<D>>> =
            object: Struct<Measurement<Quantity<D>>> {
                private val dummyMeasurement = Measurement(Quantity<D>(1.0), Time(0.0))

                override fun getTypeClass(): Class<Measurement<Quantity<D>>> = dummyMeasurement.javaClass

                override fun getTypeString(): String = "struct:QuantityMeasurement"

                override fun getSize(): Int = Struct.kSizeDouble * 2

                override fun getSchema(): String = "double value; double timestampSeconds;"

                override fun unpack(bb: ByteBuffer): Measurement<Quantity<D>> =
                    Measurement(
                        Quantity(bb.getDouble()),
                        bb.getDouble().ofUnit(seconds)
                    )

                override fun pack(bb: ByteBuffer, measurement: Measurement<Quantity<D>>) {
                    bb.putDouble(measurement.value.siValue)
                    bb.putDouble(measurement.timestamp.inUnit(seconds))
                }
            }


        fun <T> struct(baseStruct: Struct<T>, dummyValue: T): Struct<Measurement<T>> =
            object: Struct<Measurement<T>> {
                private val dummyMeasurement = Measurement(dummyValue, Time(0.0))

                override fun getTypeClass(): Class<Measurement<T>> = dummyMeasurement.javaClass

                override fun getTypeString(): String = baseStruct.typeString

                override fun getSize(): Int = baseStruct.size + Struct.kSizeDouble

                override fun getSchema(): String = baseStruct.schema + "double timestampSeconds;"

                override fun unpack(bb: ByteBuffer): Measurement<T> =
                    Measurement(
                        baseStruct.unpack(bb),
                        bb.getDouble().ofUnit(seconds)
                    )

                override fun pack(bb: ByteBuffer, measurement: Measurement<T>){
                    baseStruct.pack(bb, measurement.value)
                    bb.putDouble(measurement.timestamp.inUnit(seconds))
                }
            }
    }

    override fun equals(other: Any?): Boolean {
        return other is Measurement<*> && value == other.value && timestamp == other.timestamp
    }

    override fun hashCode(): Int {
        var result = value?.hashCode() ?: 0
        result = 31 * result + timestamp.hashCode()
        return result
    }
}

package frc.chargers.utils

import edu.wpi.first.util.struct.Struct
import java.nio.ByteBuffer

/**
 * Creates a copy of a struct, with identical schema,
 * of a type that can be seamlessly converted to a type with an existing struct.
 */
inline fun <I, reified O> createCopy(
    initial: Struct<I>,
    noinline convertor: (I) -> O,
    noinline inverseConvertor: (O) -> I
): Struct<O> =
    object: Struct<O>{
        override fun getTypeClass(): Class<O> = O::class.java

        override fun getTypeString(): String = initial.typeString

        override fun getSize(): Int = initial.size

        override fun getSchema(): String = initial.schema

        override fun unpack(byteBuffer: ByteBuffer): O = convertor(initial.unpack(byteBuffer))

        override fun pack(byteBuffer: ByteBuffer, value: O) {
            initial.pack(byteBuffer, inverseConvertor(value))
        }
    }
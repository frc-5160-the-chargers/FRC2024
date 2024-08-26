package frc.chargers.utils.struct

import java.nio.ByteBuffer

/**
 * A utility function to put a boolean to a [ByteBuffer];
 * for usage within Structs.
 */
@Suppress("unused")
fun ByteBuffer.putBoolean(value: Boolean){
    put(if(value) 1.toByte() else 0.toByte())
}
package frc.chargers.utils

import java.nio.ByteBuffer

/**
 * A utility function to get a boolean from a [ByteBuffer].
 */
@Suppress("unused")
fun ByteBuffer.getBoolean(): Boolean =
    get() == 1.toByte()

/**
 * A utility function to put a boolean to a [ByteBuffer];
 * for usage within Structs.
 */
@Suppress("unused")
fun ByteBuffer.putBoolean(value: Boolean){
    put(if(value) 1.toByte() else 0.toByte())
}
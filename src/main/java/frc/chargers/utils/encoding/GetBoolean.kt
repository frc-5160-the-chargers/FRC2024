package frc.chargers.utils.encoding

import java.nio.ByteBuffer

/**
 * A utility function to get a boolean from a [ByteBuffer].
 */
@Suppress("unused")
fun ByteBuffer.getBoolean(): Boolean =
    get() == 1.toByte()
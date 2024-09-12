@file:Suppress("unused")
package frc.chargers.utils


/**
 * Convenience object for more concisely creating arrays until
 * [KT-43871](https://youtrack.jetbrains.com/issue/KT-43871) is resolved.
 * Works by implementing the get operator, allowing usage of square brackets.
 * If possible, prefer using EJML equations for initializing matrices.
 *
 * Typical array creation:
 * ```
 * val array1D = doubleArrayOf(1.0, 2.0, 3.0)
 * val array2D =
 *     arrayOf(
 *         arrayOf(1.0, 2.0, 3.0),
 *         arrayOf(4.0, 5.0, 6.0),
 *         arrayOf(7.0, 8.0, 9.0),
 *     )
 * ```
 *
 * Usage of this class:
 * ```
 * val array1D = a[1.0, 2.0, 3.0]
 * val array2D = a[
 *                  a[1.0, 2.0, 3.0],
 *                  a[4.0, 5.0, 6.0],
 *                  a[7.0, 8.0, 9.0]
 *               ]
 * ```
 */
object a {
    inline operator fun <reified T> get(vararg elements: T): Array<T> = arrayOf(*elements)
}

/**
 * Convenience object for creating [IntArray]s, [DoubleArray]s and [FloatArray]s.
 * These arrays have no boxing overhead, but function the same as regular arrays.
 *
 * Usage:
 * ```
 * val arrOne = p[1.0,2.0,3.0]
 * val arrTwo: IntArray = p[1,2,3]
 * val arrThree = p['a','b','c']
 * ```
 * @see a
 */
object p {
    operator fun get(vararg elements: Int): IntArray = intArrayOf(*elements)
    operator fun get(vararg elements: Double): DoubleArray = doubleArrayOf(*elements)
    operator fun get(vararg elements: Byte): ByteArray = byteArrayOf(*elements)
    operator fun get(vararg elements: Float): FloatArray = floatArrayOf(*elements)
    operator fun get(vararg elements: Char): CharArray = charArrayOf(*elements)
}
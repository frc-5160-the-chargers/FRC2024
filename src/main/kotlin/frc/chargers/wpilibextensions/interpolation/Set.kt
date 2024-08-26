package frc.chargers.wpilibextensions.interpolation

import edu.wpi.first.math.interpolation.InterpolatingTreeMap

/**
 * Allows for the set operator function to be used with [InterpolatingTreeMap]s.
 */
operator fun <K,V> InterpolatingTreeMap<K,V>.set(key: K, value: V) =
    put(key, value)
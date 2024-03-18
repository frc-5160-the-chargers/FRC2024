package frc.chargers.wpilibextensions.interpolation

import edu.wpi.first.math.interpolation.InterpolatingTreeMap

operator fun <K,V> InterpolatingTreeMap<K,V>.set(key: K, value: V) =
    put(key, value)
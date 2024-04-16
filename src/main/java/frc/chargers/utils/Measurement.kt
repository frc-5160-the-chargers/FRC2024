@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils

import com.batterystaple.kmeasure.quantities.Time


/**
 * Represents a simple value accompanied by a timestamp
 *
 */
public data class Measurement<T>(
    val value: T,
    val timestamp: Time
){
    override fun equals(other: Any?): Boolean {
        return other is Measurement<*> && value == other.value && timestamp == other.timestamp
    }

    override fun hashCode(): Int {
        var result = value?.hashCode() ?: 0
        result = 31 * result + timestamp.hashCode()
        return result
    }
}

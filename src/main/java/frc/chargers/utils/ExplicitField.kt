@file:Suppress("unused")
package frc.chargers.utils

import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KMutableProperty0
import kotlin.reflect.KProperty


/*
Credits: Team 449
 */

/**
 * Represents a scope for an explicit field; not intended to be instantiated.
 */
class ExplicitFieldScope<B> internal constructor(propertyRef: KMutableProperty0<B>){
    /**
     * The backing field of a property delegated by [explicitField].
     */
    var field: B by propertyRef
}


/**
 * A property with a custom getter but a backing field of a different type.
 * @param B The type of the actual backing field
 * @param F The type of the property that is delegated
 *
 * Credits: [Team 449 repository](https://github.com/blair-robot-project/robot2024/blob/main/src/main/kotlin/frc/team449/util/Properties.kt)
 */
fun <B, F> explicitField(backing: B, get: ExplicitFieldScope<B>.() -> F) =
    object: ReadOnlyProperty<Any?, F> {
        private var backingField = backing
        private val scope = ExplicitFieldScope(::backingField)

        override fun getValue(thisRef: Any?, property: KProperty<*>): F =
            with(scope){ get() }
    }

/**
 * A property with a custom getter and setter but a backing field of a different type.
 *
 * @param B The type of the actual backing field
 * @param F The type of the property that is delegated
 *
 * Credits: [Team 449 repository](https://github.com/blair-robot-project/robot2024/blob/main/src/main/kotlin/frc/team449/util/Properties.kt)
 */
fun <B, F> explicitField(
    backing: B,
    get: ExplicitFieldScope<B>.() -> F,
    set: ExplicitFieldScope<B>.(F) -> Unit
) = object : ReadWriteProperty<Any?, F> {
        private var backingField = backing
        private val scope = ExplicitFieldScope(::backingField)

        override fun getValue(thisRef: Any?, property: KProperty<*>) = with(scope){ get() }

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: F) = with(scope) { set(value) }
    }

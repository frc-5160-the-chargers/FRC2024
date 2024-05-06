@file:Suppress("unused")
package frc.chargers.utils

import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

fun <T> ReadOnlyProperty<Any?, T>.withSetter(setter: (T) -> Unit) =
    object: ReadWriteProperty<Any?, T>{
        override fun getValue(thisRef: Any?, property: KProperty<*>): T = this@withSetter.getValue(thisRef, property)

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
            setter(value)
        }
    }

fun <T> PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, T>>.withSetter(setter: (T) -> Unit) =
    PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T>> { providedThisRef, providedProperty ->
        object: ReadWriteProperty<Any?, T> {
            private val delegate = this@withSetter.provideDelegate(providedThisRef, providedProperty)

            override fun getValue(thisRef: Any?, property: KProperty<*>): T = delegate.getValue(thisRef, property)

            override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
                setter(value)
            }
        }
    }
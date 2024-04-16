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
    PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T>> { thisRef, property ->
        object: ReadOnlyProperty<Any?, T> by this@withSetter.provideDelegate(thisRef, property),
            ReadWriteProperty<Any?, T> {
                override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
                    setter(value)
                }
        }
    }
@file:OptIn(ExperimentalTypeInference::class)
package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity

import edu.wpi.first.util.struct.StructSerializable
import frc.chargers.utils.QuantitySupplier
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import java.util.function.IntSupplier
import kotlin.experimental.ExperimentalTypeInference
import kotlin.reflect.KProperty

private fun capitalize(text: String) = text.replaceFirstChar{ it.uppercaseChar() }

/**
 * A set of property delegates that allow for automatically logged properties.
 * These delegates are backed by the DogLog library(wrapped by HorseLog).
 * Supported types include: nullables, lists, primitives, Quantities, etc.
 * Note that IntArray, DoubleArray, etc. aren't supported(no one uses those lmao)
 * ```
 * class Arm {
 *    val angle by logged{ motor.encoder.angularPosition } // a logged getter(get()) property; logged under Arm/Angle
 *    var somethingElse by logged(2.0) // a logged var
 *    var somethingElse2 by logged(2, "Arm/abcd")
 *    val getter by logged("Arm/defg"){ motor.encoder.angularVelocity }
 * }
 * ```
 */

// individual classes are used to prevent boxing overhead2
@OverloadResolutionByLambdaReturnType
@JvmName("a")
fun logged(identifier: String? = null, supplier: () -> Int) = LoggedIntGetter(identifier, supplier)
class LoggedIntGetter(private val key: String?, private val get: IntSupplier) {
    operator fun provideDelegate(thisRef: Any, property: KProperty<*>): LoggedIntGetter {
        val defaultKey = "${thisRef::class.simpleName}/${capitalize(property.name)}"
        ChargerRobot.runPeriodic { HorseLog.log(key ?: defaultKey, get.asInt) }
        return this
    }
    operator fun getValue(thisRef: Any?, property: KProperty<*>): Int = get.asInt
}

@OverloadResolutionByLambdaReturnType
@JvmName("b")
fun logged(identifier: String? = null, supplier: () -> Double) = LoggedDoubleGetter(identifier, supplier)
class LoggedDoubleGetter(private val key: String?, private val get: DoubleSupplier) {
    operator fun provideDelegate(thisRef: Any, property: KProperty<*>): LoggedDoubleGetter {
        val defaultKey = "${thisRef::class.simpleName}/${capitalize(property.name)}"
        ChargerRobot.runPeriodic { HorseLog.log(key ?: defaultKey, get.asDouble) }
        return this
    }
    operator fun getValue(thisRef: Any?, property: KProperty<*>): Double = get.asDouble
}

@OverloadResolutionByLambdaReturnType
@JvmName("c")
fun logged(identifier: String? = null, supplier: () -> Boolean) = LoggedBoolGetter(identifier, supplier)
class LoggedBoolGetter(private val key: String?, private val get: BooleanSupplier) {
    operator fun provideDelegate(thisRef: Any, property: KProperty<*>): LoggedBoolGetter {
        val defaultKey = "${thisRef::class.simpleName}/${property.name}"
        ChargerRobot.runPeriodic { HorseLog.log(key ?: defaultKey, get.asBoolean) }
        return this
    }
    operator fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = get.asBoolean
}

@OverloadResolutionByLambdaReturnType
@JvmName("d")
fun <D: AnyDimension> logged(identifier: String? = null, supplier: () -> Quantity<D>) = LoggedQuantityGetter(identifier, supplier)
class LoggedQuantityGetter<D: AnyDimension>(private val key: String?, private val get: QuantitySupplier<D>) {
    operator fun provideDelegate(thisRef: Any, property: KProperty<*>): LoggedQuantityGetter<D> {
        val defaultKey = "${thisRef::class.simpleName}/${property.name}"
        ChargerRobot.runPeriodic {
            HorseLog.log((key ?: defaultKey) + "(SI Value)", get.get().siValue)
        }
        return this
    }
    operator fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = get.get()
}



@JvmName("e") @OverloadResolutionByLambdaReturnType
fun logged(identifier: String? = null, supplier: () -> Double?) = LoggedGetter(identifier, supplier, HorseLog::logNullableDouble)
@JvmName("f") @OverloadResolutionByLambdaReturnType
fun <D: AnyDimension> logged(identifier: String? = null, supplier: () -> Quantity<D>?) = LoggedGetter(identifier, supplier, HorseLog::logNullableQuantity)
@JvmName("g") @OverloadResolutionByLambdaReturnType
fun logged(identifier: String? = null, supplier: () -> String) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("h") @OverloadResolutionByLambdaReturnType
fun <E: Enum<E>> logged(identifier: String? = null, supplier: () -> E) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("i") @OverloadResolutionByLambdaReturnType
fun <C: Collection<Int>> logged(identifier: String? = null, supplier: () -> C) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("j") @OverloadResolutionByLambdaReturnType
fun <C: Collection<Double>> logged(identifier: String? = null, supplier: () -> C) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("k") @OverloadResolutionByLambdaReturnType
fun <D: AnyDimension, C: Collection<Quantity<D>>> logged(identifier: String? = null, supplier: () -> C) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("l") @OverloadResolutionByLambdaReturnType
fun <T: StructSerializable> logged(identifier: String?, supplier: () -> T) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("m") @OverloadResolutionByLambdaReturnType
fun <C: Collection<StructSerializable>> logged(identifier: String?, supplier: () -> C) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("n") @OverloadResolutionByLambdaReturnType
fun <E: Enum<E>, C: Collection<E>> logged(identifier: String? = null, supplier: () -> C) = LoggedGetter(identifier, supplier, HorseLog::log)
@JvmName("o") @OverloadResolutionByLambdaReturnType
fun logged(identifier: String? = null, supplier: () -> Int?) = LoggedGetter(identifier, supplier, HorseLog::logNullableInt)

class LoggedGetter<T>(
    private val key: String? = null,
    private val get: () -> T,
    private val log: (String, T) -> Unit,
) {
    operator fun provideDelegate(thisRef: Any, property: KProperty<*>): LoggedGetter<T> {
        val defaultKey = "${thisRef::class.simpleName}/${property.name}"
        ChargerRobot.runPeriodic { log(key ?: defaultKey, get()) }
        return this
    }
    operator fun getValue(thisRef: Any, property: KProperty<*>) = get()
}


fun logged(value: Int, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
fun logged(value: Double, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
fun logged(value: Boolean, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
fun logged(value: String, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
fun <D: AnyDimension> logged(value: Quantity<D>, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
fun <T: StructSerializable> logged(value: T, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
fun <E: Enum<E>> logged(value: E, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)

fun logged(value: Int?, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::logNullableInt)
@JvmName("logNullable") fun logged(value: Double?, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::logNullableDouble)
@JvmName("logNullable") fun <D: AnyDimension> logged(value: Quantity<D>?, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::logNullableQuantity)
@JvmName("logNullable") fun <T: StructSerializable> logged(value: T?, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::logNullableValue)

@JvmName("logListA") fun <C: Collection<Int>> logged(value: C, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
@JvmName("logListB") fun <C: Collection<Double>> logged(value: C, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
@JvmName("logListC") fun <D: AnyDimension, C: Collection<Quantity<D>>> logged(value: C, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
@JvmName("logListD") fun <C: Collection<StructSerializable>> logged(value: C, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)
@JvmName("logListE") fun <E: Enum<E>, C: Collection<E>> logged(value: C, identifier: String? = null) = LoggedVar(value, identifier, HorseLog::log)

class LoggedVar<T>(
    private var value: T,
    private val key: String?,
    private val log: (String, T) -> Unit
) {
    operator fun provideDelegate(thisRef: Any, property: KProperty<*>): LoggedVar<T> {
        val defaultKey = "${thisRef::class.simpleName}/${property.name}"
        ChargerRobot.runPeriodic { log(key ?: defaultKey, value) }
        return this
    }
    operator fun getValue(thisRef: Any, property: KProperty<*>): T = value
    operator fun setValue(thisRef: Any, property: KProperty<*>, value: T) { this.value = value }
}
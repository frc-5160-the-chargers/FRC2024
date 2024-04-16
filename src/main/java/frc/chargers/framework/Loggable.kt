package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.util.datalog.*
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.wpilibj.DataLogManager
import java.util.WeakHashMap
import kotlin.experimental.ExperimentalTypeInference
import kotlin.internal.LowPriorityInOverloadResolution
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty


private val allLogEntries: MutableMap<String, DataLogEntry> = WeakHashMap()
private val previousLoggedValues: MutableMap<String, Any?> = WeakHashMap()

private fun <E : DataLogEntry> getEntry(
    identifier: String,
    entryCreator: (DataLog, String) -> E
): E? {
    var entry = allLogEntries[identifier]
    if (entry != null) {
        // as? is a safe cast that returns null on failure
        try{
            @Suppress("UNCHECKED_CAST") // try and except will catch a casting fail
            return entry as E?
        }catch(e: ClassCastException){
            println("It seems that you are logging 2 different types within the same field.")
            return null
        }
    }

    entry = entryCreator(DataLogManager.getLog(), identifier)
    allLogEntries[identifier] = entry
    return entry
}


/**
 * Represents a class that can be logged.
 *
 * Values can be logged in 2 ways: through the explicit logging function "log",
 * or through the property delegate provider function "logged".
 *
 * Example usage of this interface:
 *
 * ```
 * class Arm: Loggable{
 *      override val logGroup = "Arm"
 *
 *      init{
 *          // manual logging; will log to "Arm/atSetpoint" once
 *          log("atSetpoint", false)
 *      }
 *
 *      // automatic logging; will be logged to "Arm/value" repeatedly.
 *      // value will act as a regular getter, but the values will be logged as well.
 *      val value by logged{ 5.0 }
 *
 *      // will be logged to "Arm/Position(SI Value)" repeatedly
 *      // as the getter returns a Quantity<D>.
 *      val value2 by logged("Position"){ motor.encoder.angularVelocity }
 *
 *      // will be logged to "Arm/value3" when necessary.
 *      // acts as a regular mutable property, with logging.
 *      var value3 by logged(5.0)
 *
 *      // for variable delegates, the initial value goes before the log namespace
 *      // due to overload resolution reasons.
 *      var value4 by logged(5.0, "Hi")
 *
 *      // Structs are also supported
 *      val value5 by logged(UnitPose2d.struct){ drivetrain.pose }
 * }
 */
@Suppress("unused")
@OptIn(ExperimentalTypeInference::class)
interface Loggable {
    /**
     * The namespace in which to log to.
     */
    val logGroup: String




    fun log(identifier: String, value: Int) {
        if (previousLoggedValues[identifier] != value){
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::IntegerLogEntry)?.append(value.toLong())
        }
    }

    fun log(identifier: String, value: Double) {
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::DoubleLogEntry)?.append(value)
        }
    }

    fun log(identifier: String, value: Quantity<*>){
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            log("$logGroup/$identifier(SI Value)", value.siValue)
        }
    }

    fun log(identifier: String, value: Boolean) {
        if (previousLoggedValues[identifier] != value){
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::BooleanLogEntry)?.append(value)
        }
    }

    fun log(identifier: String, value: String) {
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::StringLogEntry)?.append(value)
        }
    }

    fun <T> log(struct: Struct<T>, identifier: String, value: T) {
        DataLogManager.getLog().addSchema(struct)
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier") { log: DataLog?, k: String? ->
                StructLogEntry.create(log, k, struct)
            }?.append(value)
        }
    }




    fun log(identifier: String, value: Collection<Int>) {
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::IntegerArrayLogEntry)?.append(value.map { it.toLong() }.toLongArray())
        }
    }

    fun log(identifier: String, value: Collection<Double>) {
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::DoubleArrayLogEntry)?.append(value.toDoubleArray())
        }
    }

    fun log(identifier: String, value: Collection<Boolean>){
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::BooleanArrayLogEntry)?.append(value.toBooleanArray())
        }
    }

    fun log(identifier: String, value: Collection<String>) {
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier", ::StringArrayLogEntry)?.append(value.toTypedArray())
        }
    }

    fun log(identifier: String, value: Collection<Quantity<*>>) {
        log("$logGroup/$identifier", value.map{ it.siValue })
    }

    fun <T> log(struct: Struct<T>, identifier: String, value: Collection<T>) {
        DataLogManager.getLog().addSchema(struct)
        if (previousLoggedValues[identifier] != value) {
            previousLoggedValues[identifier] = value
            getEntry("$logGroup/$identifier") { log: DataLog?, k: String? ->
                StructArrayLogEntry.create(log, k, struct)
            }?.append(value)
        }
    }




    @LowPriorityInOverloadResolution
    fun log(identifier: String, value: Int?){
        log("$logGroup/$identifier/value", value ?: 0)
        log("$logGroup/$identifier/isValid", value != null)
    }

    @LowPriorityInOverloadResolution
    fun log(identifier: String, value: Double?){
        log("$logGroup/$identifier/value", value ?: 0.0)
        log("$logGroup/$identifier/isValid", value != null)
    }

    @LowPriorityInOverloadResolution
    fun log(identifier: String, value: Quantity<*>?){
        log("$logGroup/$identifier/value(SI value)", value?.siValue ?: 0.0)
        log("$logGroup/$identifier/isValid", value != null)
    }

    @LowPriorityInOverloadResolution
    fun <T> log(struct: Struct<T>, identifier: String, value: T?) {
        if (value != null){
            log(struct, "$logGroup/$identifier/value", value)
        }
        log("$logGroup/$identifier/isValid", value != null)
    }





    @OverloadResolutionByLambdaReturnType
    fun logged(identifier: String? = null, supplier: () -> Int) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Int>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Int>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    fun logged(identifier: String? = null, supplier: () -> Double) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Double>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Double>{ _, _ -> supplier() }
        }


    @OverloadResolutionByLambdaReturnType
    fun logged(identifier: String? = null, supplier: () -> Boolean) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Boolean>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Boolean>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    fun logged(identifier: String? = null, supplier: () -> String) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, String>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, String>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    fun <D: AnyDimension> logged(identifier: String? = null, supplier: () -> Quantity<D>) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Quantity<D>>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Quantity<D>>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    fun <T> logged(struct: Struct<T>, identifier: String? = null, supplier: () -> T) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, T>> { _, property ->
            ChargerRobot.runPeriodically {
                log(struct, logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, T>{ _, _ -> supplier() }
        }




    @OverloadResolutionByLambdaReturnType
    fun <C: Collection<Int>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }

            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    fun <C: Collection<Double>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }

            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    fun <C: Collection<Boolean>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }

            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }


    @OverloadResolutionByLambdaReturnType
    fun <C: Collection<String>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }

            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }


    @OverloadResolutionByLambdaReturnType
    fun <D: AnyDimension, C: Collection<Quantity<D>>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }

            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    fun <T, C: Collection<T>> logged(struct: Struct<T>, identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>> { _, property ->
            ChargerRobot.runPeriodically {
                log(struct, logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, C>{ _, _ -> supplier() }
        }




    @OverloadResolutionByLambdaReturnType
    @LowPriorityInOverloadResolution
    fun logged(identifier: String? = null, supplier: () -> Int?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Int?>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Int?>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @LowPriorityInOverloadResolution
    fun logged(identifier: String? = null, supplier: () -> Double?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Double?>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Double?>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @LowPriorityInOverloadResolution
    fun <D: AnyDimension> logged(identifier: String? = null, supplier: () -> Quantity<D>?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Quantity<D>?>> { _, property ->
            ChargerRobot.runPeriodically {
                log(logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Quantity<D>?>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @LowPriorityInOverloadResolution
    fun <T> logged(struct: Struct<T>, identifier: String? = null, supplier: () -> T?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, T?>> { _, property ->
            ChargerRobot.runPeriodically {
                log(struct, logGroup + "/" + (identifier ?: property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, T?>{ _, _ -> supplier() }
        }




    class LoggedMutableProperty<T>(
        private val identifier: String?,
        private var value: T,
        private val loggerFunction: (String, T) -> Unit
    ): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T>> {
        override fun provideDelegate(thisRef: Any?, property: KProperty<*>) =
            object: ReadWriteProperty<Any?, T>{
                init{
                    loggerFunction(identifier ?: property.name, value)
                }

                override fun getValue(thisRef: Any?, property: KProperty<*>): T = value

                override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
                    this@LoggedMutableProperty.value = value
                    loggerFunction(identifier ?: property.name, value)
                }
            }
    }

    fun logged(value: Int, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Int>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun logged(value: Double, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Double>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun logged(value: Boolean, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Boolean>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun logged(value: String, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, String>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <D: AnyDimension> logged(value: Quantity<D>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Quantity<D>>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <T> logged(value: T, struct: Struct<T>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T>> =
        LoggedMutableProperty(identifier, value){ namespace, logValue -> log(struct, namespace, logValue) }




    fun <C: Collection<Int>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <C: Collection<Double>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <C: Collection<Boolean>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <D: AnyDimension, C: Collection<Quantity<D>>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <T, C: Collection<T>> logged(value: C, struct: Struct<T>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value){ namespace, logValue -> log(struct, namespace, logValue) }




    @LowPriorityInOverloadResolution
    fun logged(value: Int?, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Int?>> =
        LoggedMutableProperty(identifier, value, ::log)

    @LowPriorityInOverloadResolution
    fun logged(value: Double?, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Double?>> =
        LoggedMutableProperty(identifier, value, ::log)

    @LowPriorityInOverloadResolution
    fun <D: AnyDimension> logged(value: Quantity<D>?, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Quantity<D>?>> =
        LoggedMutableProperty(identifier, value, ::log)

    @LowPriorityInOverloadResolution
    fun <T> logged(value: T?, struct: Struct<T>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T?>> =
        LoggedMutableProperty(identifier, value){ namespace, logValue -> log(struct, namespace, logValue) }
}
package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.networktables.*
import edu.wpi.first.util.datalog.*
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import java.util.*
import kotlin.experimental.ExperimentalTypeInference
import kotlin.internal.LowPriorityInOverloadResolution
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

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
@Suppress("INAPPLICABLE_JVM_NAME", "unused")
@OptIn(ExperimentalTypeInference::class)
interface Loggable {
    /**
     * The namespace in which to log to.
     */
    val namespace: String

    companion object {
        private var fileOnly: Boolean = false
        init {
            ChargerRobot.runPeriodic{ if (DriverStation.isFMSAttached()) fileOnly = true }
        }
        // datalog-related storage
        private val dataLogEntries: MutableMap<String, DataLogEntry> = WeakHashMap()
        // networktables-related storage
        private val ntInstance by lazy{ NetworkTableInstance.getDefault() }
        private val ntPublishers: MutableMap<String, Publisher> = HashMap()

        // fetches a data log entry; creating a new one if not present
        private fun <E : DataLogEntry> getEntry(
            identifier: String,
            entryCreator: (DataLog, String) -> E
        ): E? {
            var entry = dataLogEntries[identifier]
            if (entry != null) {
                // as? is a safe cast that returns null on failure
                try{
                    @Suppress("UNCHECKED_CAST") // try and except will catch a casting fail
                    val newEntry = entry as E?
                    newEntry.toString() // runs a random function to ensure that if casting fails, then an exception will be thrown
                    return newEntry
                }catch(e: ClassCastException){
                    println("It seems that you are logging 2 different types within the same field.")
                    return null
                }
            }

            entry = entryCreator(DataLogManager.getLog(), identifier)
            dataLogEntries[identifier] = entry
            return entry
        }

        // capitalization
        private fun capitalize(input: String): String =
            input.replaceFirstChar{ if (it.isLowerCase()) it.uppercaseChar() else it }
    }

    /**
     * Logs the latency of a certain function.
     * There is no runtime overhead for logging latency like this.
     *
     * @param identifier: The namespace of which latency is logged.
     * @param function: The function that will be logged.
     */
    fun <T> logLatency(identifier: String, function: () -> T): T {
        val startTime = Timer.getFPGATimestamp()
        val returnValue = function()
        log("$identifier(MS)", (Timer.getFPGATimestamp() - startTime) * 1000)
        return returnValue
    }

    // The functions below log basic values manually.
    // This includes: Int, Boolean, Double, Quantity<*>, and struct serializable types.
    @JvmName("logInt")
    fun log(identifier: String, value: Int) {
        getEntry("$namespace/$identifier", ::IntegerLogEntry)?.append(value.toLong())
        if (!fileOnly){
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getIntegerTopic(k).publish()
            } as IntegerPublisher)
                .set(value.toLong())
        }
    }

    @JvmName("logDouble")
    fun log(identifier: String, value: Double) {
        getEntry("$namespace/$identifier", ::DoubleLogEntry)?.append(value)
        if (!fileOnly){
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getDoubleTopic(k).publish()
            } as DoublePublisher)
                .set(value)
        }
    }

    @JvmName("logQuantity")
    fun log(identifier: String, value: Quantity<*>){
        log("$identifier(SI Value)", value.siValue)
    }

    @JvmName("logBool")
    fun log(identifier: String, value: Boolean) {
        getEntry("$namespace/$identifier", ::BooleanLogEntry)?.append(value)
        if (!fileOnly){
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getBooleanTopic(k).publish()
            } as BooleanPublisher)
                .set(value)
        }
    }

    @JvmName("logString")
    fun log(identifier: String, value: String) {
        getEntry("$namespace/$identifier", ::StringLogEntry)?.append(value)
        if (!fileOnly){
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getStringTopic(k).publish()
            } as StringPublisher)
                .set(value)
        }
    }

    @JvmName("logEnum")
    fun log(identifier: String, value: Enum<*>) {
        log(identifier, value.name)
    }

    @JvmName("logStructable")
    fun <T> log(struct: Struct<T>, identifier: String, value: T) {
        DataLogManager.getLog().addSchema(struct)
        getEntry("$namespace/$identifier") { log: DataLog?, k: String ->
            StructLogEntry.create(log, k, struct)
        }?.append(value)
        if (!fileOnly){
            ntInstance.addSchema(struct)
            @Suppress("UNCHECKED_CAST")
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getStructTopic(k, struct).publish()
            } as StructPublisher<T>)
                .set(value)
        }
    }

    // The functions below log Collections of values individually.
    // Collection types include Array<*>, List<*>, MutableList<*>, etc.
    // This includes: Int, Boolean, Double, Quantity<*>, and struct serializable types.
    @JvmName("logIntList")
    fun log(identifier: String, value: Collection<Int>) {
        getEntry("$namespace/$identifier", ::IntegerArrayLogEntry)?.append(value.map { it.toLong() }.toLongArray())
        if (!fileOnly){
            // NT backend only supports int64[], so we have to manually widen to 64 bits before sending
            val widened = value.map{ it.toLong() }.toLongArray()
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getIntegerArrayTopic(k).publish()
            } as IntegerArrayPublisher)
                .set(widened)
        }
    }

    @JvmName("logDoubleList")
    fun log(identifier: String, value: Collection<Double>) {
        getEntry("$namespace/$identifier", ::DoubleArrayLogEntry)?.append(value.toDoubleArray())
        if (!fileOnly){
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getDoubleArrayTopic(k).publish()
            } as DoubleArrayPublisher)
                .set(value.toDoubleArray())
        }
    }

    @JvmName("logBoolList")
    fun log(identifier: String, value: Collection<Boolean>){
        getEntry("$namespace/$identifier", ::BooleanArrayLogEntry)?.append(value.toBooleanArray())
        if (!fileOnly){
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getBooleanArrayTopic(k).publish()
            } as BooleanArrayPublisher)
                .set(value.toBooleanArray())
        }
    }

    @JvmName("logStringList")
    fun log(identifier: String, value: Collection<String>) {
        getEntry("$namespace/$identifier", ::StringArrayLogEntry)?.append(value.toTypedArray())
        if (!fileOnly){
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getStringArrayTopic(k).publish() } as StringArrayPublisher
                    )
                .set(value.toTypedArray())
        }
    }

    @JvmName("logQuantityList")
    fun log(identifier: String, value: Collection<Quantity<*>>) {
        log(identifier, value.map{ it.siValue })
    }

    @JvmName("logStructableList")
    fun <T> log(struct: Struct<T>, identifier: String, value: Collection<T>) {
        DataLogManager.getLog().addSchema(struct)
        getEntry("$namespace/$identifier") { log: DataLog?, k: String ->
            StructArrayLogEntry.create(log, k, struct)
        }?.append(value)

        if (!fileOnly){
            val valueAsArray = Array<Any?>(value.size) {}
            val valueAsList = value.toList()
            for (i in value.indices){
                valueAsArray[i] = valueAsList[i]
            }
            @Suppress("UNCHECKED_CAST")
            (ntPublishers.computeIfAbsent("$namespace/$identifier") {
                    k: String -> ntInstance.getStructArrayTopic(k, struct).publish()
            } as StructArrayPublisher<T>)
                .set(valueAsArray as Array<T>)
        }
    }

    // The functions below log nullable values manually.
    // This includes: Int, Double?, Quantity<*>?, and T?, where T has a corresponding struct.
    @LowPriorityInOverloadResolution
    @JvmName("logNullableInt")
    fun log(identifier: String, value: Int?){
        log("$identifier/value", value ?: 0)
        log("$identifier/isValid", value != null)
    }

    @LowPriorityInOverloadResolution
    @JvmName("logNullableDouble")
    fun log(identifier: String, value: Double?){
        log("$identifier/value", value ?: 0.0)
        log("$identifier/isValid", value != null)
    }

    @LowPriorityInOverloadResolution
    @JvmName("logNullableQuantity")
    fun log(identifier: String, value: Quantity<*>?){
        log("$identifier/value(SI value)", value?.siValue ?: 0.0)
        log("$identifier/isValid", value != null)
    }

    @LowPriorityInOverloadResolution
    @JvmName("logNullableStructable")
    fun <T> log(struct: Struct<T>, identifier: String, value: T?) {
        if (value != null){
            log(struct, "$identifier/value", value)
        }
        log("$identifier/isValid", value != null)
    }




    // The functions below provide property delegates that act as getters for nullable types,
    // but automatically log their getter value every loop.
    // Note: these functions have to come first before their non-nullable variants;
    // otherwise, the compiler has trouble determining the correct types.
    // The list of types matches the manually loggable nullable types.
    @OverloadResolutionByLambdaReturnType
    @JvmName("nullableIntLoggedDelegate")
    fun logged(identifier: String? = null, supplier: () -> Int?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Int?>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Int?>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("nullableDoubleLoggedDelegate")
    fun logged(identifier: String? = null, supplier: () -> Double?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Double?>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Double?>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("nullableQuantityLoggedDelegate")
    fun <D: AnyDimension> logged(identifier: String? = null, supplier: () -> Quantity<D>?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Quantity<D>?>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Quantity<D>?>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("nullableStructableLoggedDelegate")
    fun <T> logged(struct: Struct<T>, identifier: String? = null, supplier: () -> T?) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, T?>> { _, property ->
            ChargerRobot.runPeriodic {
                log(struct, identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, T?>{ _, _ -> supplier() }
        }




    // The functions below provide property delegates that act as getters for basic types,
    // but automatically log their getter value every loop.
    // The list of types matches the manually-loggable types.
    @OverloadResolutionByLambdaReturnType
    @JvmName("intLoggedDelegate")
    fun logged(identifier: String? = null, supplier: () -> Int) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Int>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Int>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("doubleLoggedDelegate")
    fun logged(identifier: String? = null, supplier: () -> Double) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Double>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Double>{ _, _ -> supplier() }
        }


    @OverloadResolutionByLambdaReturnType
    @JvmName("boolLoggedDelegate")
    fun logged(identifier: String? = null, supplier: () -> Boolean) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Boolean>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Boolean>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("stringLoggedDelegate")
    fun logged(identifier: String? = null, supplier: () -> String) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, String>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, String>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("enumLoggedDelegate")
    fun <E: Enum<E>> logged(identifier: String? = null, supplier: () -> E) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, E>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, E>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("quantityLoggedDelegate")
    fun <D: AnyDimension> logged(identifier: String? = null, supplier: () -> Quantity<D>) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Quantity<D>>> { _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, Quantity<D>>{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("structableLoggedDelegate")
    fun <T> logged(struct: Struct<T>, identifier: String? = null, supplier: () -> T) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, T>> { _, property ->
            ChargerRobot.runPeriodic {
                log(struct, identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, T>{ _, _ -> supplier() }
        }




    // The functions below provide property delegates that act as getters for Collection types,
    // but automatically log their getter value every loop.
    // Collection types include Array<*>, List<*>, MutableList<*>, etc.
    // The list of types matches the manually-loggable types.
    @OverloadResolutionByLambdaReturnType
    @JvmName("intListLoggedDelegate")
    fun <C: Collection<Int>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("doubleListLoggedDelegate")
    fun <C: Collection<Double>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("boolListLoggedDelegate")
    fun <C: Collection<Boolean>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }


    @OverloadResolutionByLambdaReturnType
    @JvmName("stringListLoggedDelegate")
    fun <C: Collection<String>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }


    @OverloadResolutionByLambdaReturnType
    @JvmName("quantityListLoggedDelegate")
    fun <D: AnyDimension, C: Collection<Quantity<D>>> logged(identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>>{ _, property ->
            ChargerRobot.runPeriodic {
                log(identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty{ _, _ -> supplier() }
        }

    @OverloadResolutionByLambdaReturnType
    @JvmName("structableListLoggedDelegate")
    fun <T, C: Collection<T>> logged(struct: Struct<T>, identifier: String? = null, supplier: () -> C) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, C>> { _, property ->
            ChargerRobot.runPeriodic {
                log(struct, identifier ?: capitalize(property.name), supplier())
            }
            return@PropertyDelegateProvider ReadOnlyProperty<Any?, C>{ _, _ -> supplier() }
        }



    class LoggedMutableProperty<T>(
        private val identifier: String?,
        private var value: T,
        private val loggerFunction: (String, T) -> Unit
    ): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T>> {
        override fun provideDelegate(thisRef: Any?, property: KProperty<*>) =
            object: ReadWriteProperty<Any?, T>{
                private val propertyName = capitalize(property.name)

                init{
                    loggerFunction(identifier ?: propertyName, value)
                }

                override fun getValue(thisRef: Any?, property: KProperty<*>): T = value

                override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
                    this@LoggedMutableProperty.value = value
                    loggerFunction(identifier ?: propertyName, value)
                }
            }
    }

    // The functions below provide property delegates that act as regular, mutable variables for basic types,
    // but automatically log their value every time they are set(and once on initialization).
    fun logged(value: Int, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Int>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun logged(value: Double, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Double>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun logged(value: Boolean, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Boolean>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun logged(value: String, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, String>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <E: Enum<E>> logged(value: E, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, E>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <D: AnyDimension> logged(value: Quantity<D>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Quantity<D>>> =
        LoggedMutableProperty(identifier, value, ::log)

    fun <T> logged(value: T, struct: Struct<T>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T>> =
        LoggedMutableProperty(identifier, value){ namespace, logValue -> log(struct, namespace, logValue) }




    // The functions below provide property delegates that act as regular, mutable variables for Collection types,
    // but automatically log their value every time they are set(and once on initialization).
    @JvmName("loggedIntListDelegate")
    fun <C: Collection<Int>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    @JvmName("loggedDoubleListDelegate")
    fun <C: Collection<Double>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    @JvmName("loggedBoolListDelegate")
    fun <C: Collection<Boolean>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    @JvmName("loggedQuantityListDelegate")
    fun <D: AnyDimension, C: Collection<Quantity<D>>> logged(value: C, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value, ::log)

    @JvmName("loggedStructableListDelegate")
    fun <T, C: Collection<T>> logged(value: C, struct: Struct<T>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, C>> =
        LoggedMutableProperty(identifier, value){ namespace, logValue -> log(struct, namespace, logValue) }




    // The functions below provide property delegates that act as regular, mutable variables for nullable types,
    // but automatically log their value every time they are set(and once on initialization).
    @LowPriorityInOverloadResolution
    @JvmName("loggedNullableIntDelegate")
    fun logged(value: Int?, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Int?>> =
        LoggedMutableProperty(identifier, value, ::log)

    @LowPriorityInOverloadResolution
    @JvmName("loggedNullableDoubleDelegate")
    fun logged(value: Double?, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Double?>> =
        LoggedMutableProperty(identifier, value, ::log)

    @LowPriorityInOverloadResolution
    @JvmName("loggedNullableQuantityDelegate")
    fun <D: AnyDimension> logged(value: Quantity<D>?, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, Quantity<D>?>> =
        LoggedMutableProperty(identifier, value, ::log)

    @LowPriorityInOverloadResolution
    @JvmName("loggedNullableStructDelegate")
    fun <T> logged(value: T?, struct: Struct<T>, identifier: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?, T?>> =
        LoggedMutableProperty(identifier, value){ namespace, logValue -> log(struct, namespace, logValue) }
}
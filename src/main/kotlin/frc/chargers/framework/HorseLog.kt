package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Quantity
import dev.doglog.DogLog
import dev.doglog.DogLogOptions
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.Timer


/**
 * A small [DogLog] wrapper that adds kotlin-specific methods.
 *
 * This is used the same as [DogLog]; however, it supports kotlin nullables,
 * adds support for [Collection]s(aka Lists, MutableLists, etc.),
 * and supports logging kmeasure [Quantity]s.
 * ```
 * import frc.chargers.framework.HorseLog.log // this singleton is designed to be statically imported in subsystems
 *
 * log("Angle", 30.degrees) // will log as "Angle(SI Value)", 0.68(value in radians)
 * log("values", listOf(1.0, 2.0, 3.0))
 * logNullableDouble("Hello", 2.0)
 * logNullableInt("Bye", 5)
 * HorseLog.log("Angle", 30.degrees) // also valid
 * ```
 */
@Suppress("unused")
object HorseLog {
    /* Ported from DogLog */
    fun log(key: String, value: Long) = DogLog.log(key, value)
    fun log(key: String, value: Double) = DogLog.log(key, value)
    fun log(key: String, value: Boolean) = DogLog.log(key, value)
    fun log(key: String, value: Enum<*>) = DogLog.log(key, value)
    fun log(key: String, value: String?) = DogLog.log(key, value ?: "NOTHING")
    fun <T: StructSerializable> log(key: String, value: T) = DogLog.log(key, value)

    fun log(key: String, value: IntArray) = DogLog.log(key, value)
    fun log(key: String, value: DoubleArray) = DogLog.log(key, value)
    fun log(key: String, value: BooleanArray) = DogLog.log(key, value)

    fun logFault(faultName: String) = DogLog.logFault(faultName)
    fun logFault(faultName: Enum<*>) = DogLog.logFault(faultName)

    fun getOptions() = DogLog.getOptions()
    fun setOptions(options: DogLogOptions) = DogLog.setOptions(options)
    fun setEnabled(newEnabled: Boolean) = DogLog.setEnabled(newEnabled)
    fun setPdh(pdh: PowerDistribution) = DogLog.setPdh(pdh)

    /* New methods */
    /**
     * Logs the latency of a certain function.
     * There is no runtime overhead for logging latency like this.
     *
     * @param identifier: The namespace of which latency is logged.
     * @param function: The function that will be logged.
     */
    inline fun <T> logLatency(identifier: String, function: () -> T): T {
        val startTime = Timer.getFPGATimestamp()
        val returnValue = function()
        log("$identifier(MS)", (Timer.getFPGATimestamp() - startTime) * 1000)
        return returnValue
    }

    @JvmName("Log0") fun log(key: String, value: Quantity<*>) = DogLog.log("$key(SI Value)", value.siValue)
    fun log(key: String, value: Int) = DogLog.log(key, value.toLong())

    fun logNullableInt(key: String, value: Int?) = logNullableImpl(key, value, ::log)
    fun logNullableDouble(key: String, value: Double?) = logNullableImpl(key, value, ::log)
    fun logNullableQuantity(key: String, value: Quantity<*>?) = logNullableImpl(key, value, ::log)
    fun logNullableValue(key: String, value: StructSerializable?) = logNullableImpl(key, value, ::log)

    @JvmName("L1") fun log(key: String, value: Collection<Int>) = DogLog.log(key, value.toIntArray())
    @JvmName("L2") fun log(key: String, value: Collection<Double>) = DogLog.log(key, value.toDoubleArray())
    @JvmName("L3") fun log(key: String, value: Collection<String>) = DogLog.log(key, value.toTypedArray())
    @JvmName("L4") fun log(key: String, value: Collection<Enum<*>>) = DogLog.log(key, value.toTypedArray())
    @JvmName("L5") fun log(key: String, value: Collection<Quantity<*>>) = DogLog.log("$key(SI Value)", value.map{ it.siValue }.toDoubleArray())
    @JvmName("L6") inline fun <reified T: StructSerializable> log(key: String, value: Collection<T>) = DogLog.log(key, value.toTypedArray())

    private inline fun <T> logNullableImpl(key: String, value: T?, logRegular: (String, T & Any) -> Unit){
        if (value == null){
            DogLog.log("$key/isPresent", false)
        } else {
            DogLog.log("$key/isPresent", true)
            logRegular("$key/$value", value)
        }
    }
}
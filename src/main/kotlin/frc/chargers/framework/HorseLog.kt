package frc.chargers.framework

import com.batterystaple.kmeasure.quantities.Quantity
import dev.doglog.DogLog
import dev.doglog.DogLogOptions
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.datalog.StringLogEntry
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.*
import elastic.Elastic
import elastic.Elastic.ElasticNotification.NotificationLevel

/**
 * A small [DogLog] wrapper that adds kotlin-specific methods
 * for logging nullables, [List]s and KMeasure [Quantity]s.
 * In addition, it adds support for an integrated alert-logging system via logError, logWarning and logInfo.
 * ```
 * import frc.chargers.framework.HorseLog.log // this singleton is designed to be statically imported in subsystems
 * import frc.chargers.framework.HorseLog.logError
 * import frc.chargers.framework.HorseLog // alternative
 *
 * log("AngleValue", 30.degrees) // will log as "AngleValue, 0.68"(value in radians)
 * logError("Title", "Description")
 * logNullableDouble("Hello", null)
 * HorseLog.log("Angle", 30.degrees) // also valid
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
    fun log(key: String, value: LongArray) = DogLog.log(key, value)
    fun log(key: String, value: Array<String>) = DogLog.log(key, value)
    fun log(key: String, value: Array<Enum<*>>) = DogLog.log(key, value)
    fun <T: StructSerializable> log(key: String, value: Array<T>) = DogLog.log(key, value)

    fun faultsLogged() = DogLog.faultsLogged()
    fun getOptions(): DogLogOptions = DogLog.getOptions()
    fun setOptions(options: DogLogOptions) = DogLog.setOptions(options)
    fun setEnabled(newEnabled: Boolean) = DogLog.setEnabled(newEnabled)
    fun setPdh(pdh: PowerDistribution) = DogLog.setPdh(pdh)

    /* New methods */
    var logToConsole = false

    fun logError(title: String, description: Any) {
        val str = "(ERROR) $title: $description"
        if (RobotBase.isSimulation()) error(str)
        if (logToConsole) DriverStation.reportError(str, true)
        DogLog.logFault(str)
        elasticAlert(NotificationLevel.ERROR, title, description.toString())
    }

    fun logWarning(title: String, description: Any) {
        val str = "(Warning) $title: $description"
        if (logToConsole) DriverStation.reportWarning(str, false)
        DogLog.logFault(str)
        elasticAlert(NotificationLevel.WARNING, title, description.toString())
    }

    fun logInfo(title: String, description: Any) {
        if (logToConsole) println("$title: $description")
        elasticAlert(NotificationLevel.INFO, title, description.toString())
    }

    /** Logs to the metadata tab of advantagescope. */
    fun logMetadata(title: String, description: Any) {
        NetworkTableInstance.getDefault()
            .getStringTopic("/Metadata/$title")
            .publish()
            .set(description.toString())
        StringLogEntry(DataLogManager.getLog(), "/Metadata/$title")
            .append(description.toString())
    }

    private val cache = mutableMapOf<String, String>() // ensures that no spamming occurs
    private val internalNotif = Elastic.ElasticNotification(NotificationLevel.INFO, "", "")
    private fun elasticAlert(level: NotificationLevel, title: String, description: String) {
        if (cache[description] == title) return
        cache[description] = title
        internalNotif.level = level
        internalNotif.title = title
        internalNotif.description = description
        Elastic.sendAlert(internalNotif)
    }

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

    fun log(key: String, value: Int) = DogLog.log(key, value.toLong())
    @JvmName("Log0") fun log(key: String, value: Quantity<*>) = DogLog.log(key, value.siValue)

    fun logNullableInt(key: String, value: Int?) = logNullableImpl(key, value, ::log)
    fun logNullableDouble(key: String, value: Double?) = logNullableImpl(key, value, ::log)
    fun logNullableQuantity(key: String, value: Quantity<*>?) = logNullableImpl(key, value, ::log)
    fun <T: StructSerializable> logNullableValue(key: String, value: T?) = logNullableImpl(key, value, ::log)

    @JvmName("L1") fun log(key: String, value: List<Int>) = DogLog.log(key, value.toIntArray())
    @JvmName("L2") fun log(key: String, value: List<Double>) = logDoubleListImpl(key, value) { it }
    @JvmName("L3") fun log(key: String, value: List<String>) = DogLog.log(key, value.toTypedArray())
    @JvmName("L4") fun log(key: String, value: List<Enum<*>>) = DogLog.log(key, value.toTypedArray())
    @JvmName("L5") fun log(key: String, value: List<Quantity<*>>) = logDoubleListImpl(key, value){ it.siValue }
    @JvmName("L6") inline fun <reified T: StructSerializable> log(key: String, value: List<T>) = DogLog.log(key, value.toTypedArray())

    private inline fun <T> logNullableImpl(key: String, value: T?, logRegular: (String, T) -> Unit){
        if (value == null){
            DogLog.log("$key/isPresent", false)
        } else {
            DogLog.log("$key/isPresent", true)
            logRegular("$key/value", value)
        }
    }
    private val doubleArrayStore = mutableMapOf<String, DoubleArray>()
    private inline fun <T> logDoubleListImpl(key: String, value: List<T>, mapper: (T) -> Double) {
        val prevValue = doubleArrayStore[key]
        if (prevValue == null || prevValue.size != value.size) {
            DogLog.log(key, value.map(mapper).toDoubleArray().also{ doubleArrayStore[key] = it })
        } else {
            for (i in prevValue.indices) { prevValue[i] = mapper(value[i]) }
            DogLog.log(key, prevValue)
        }
    }
    @Deprecated("Use logError and logWarning instead.") fun logFault(faultName: String) = DogLog.logFault(faultName)
    @Deprecated("Use logError and logWarning instead.") fun logFault(faultName: Enum<*>) = DogLog.logFault(faultName)
}
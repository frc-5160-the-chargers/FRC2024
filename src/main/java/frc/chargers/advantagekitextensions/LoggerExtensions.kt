@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.advantagekitextensions

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.micro
import com.batterystaple.kmeasure.units.milli
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.framework.ChargerRobot
import org.littletonrobotics.junction.Logger.*


private const val warningMsg =
    "Hmm... Values are attempting to be logged, but the log table has not been initialized. Are you using the ChargerRobot class?"

/**
 * Records the output of a generic [Quantity].
 */
public fun <D: Dimension<*,*,*,*>> recordOutput(key: String, value: Quantity<D>){
    recordOutput("$key(SI Value)", value.siValue)
}

/**
 * Records the output of a generic [AdvantageKitLoggable].
 */
public fun <T: AdvantageKitLoggable<T>> recordOutput(key: String, value: T){
    val logTable = if (hasReplaySource()) ChargerRobot.AK_LOGGABLE_REPLAY_TABLE else ChargerRobot.AK_LOGGABLE_REAL_TABLE
    if (logTable != null){
        value.pushToLog(logTable, key)
    }else{
        println(warningMsg)
    }
}

/**
 * Records the output of multiple [AdvantageKitLoggable]s.
 */
public fun <T: AdvantageKitLoggable<T>> recordOutput(key: String, values: Collection<T>){
    val logTable = if (hasReplaySource()) ChargerRobot.AK_LOGGABLE_REPLAY_TABLE else ChargerRobot.AK_LOGGABLE_REAL_TABLE
    val allItems = values.toList()
    if (logTable != null){
        for (i in allItems.indices){
            allItems[i].pushToLog(logTable, "$key/$i")
        }
    }else{
        println(warningMsg)
    }
}

/**
 * Records the output of multiple [AdvantageKitLoggable]s.
 */
public fun <T: AdvantageKitLoggable<T>> recordOutput(key: String, vararg values: T) =
    recordOutput(key, values.toList())



/**
 * Runs a code block while logging & returning its latency.
 */
public inline fun recordLatency(logName: String? = null, toRun: () -> Unit): Time{
    val startTime = getRealTimestamp().ofUnit(micro.seconds)
    toRun()
    val dt = (getRealTimestamp().ofUnit(micro.seconds) - startTime)
    if (logName != null){
        recordOutput("$logName(ms)", dt.inUnit(milli.seconds))
    }
    return dt
}





@file:Suppress("unused")
package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.reflect.KProperty

/**
 * A property delegate for a [Double] that can be tuned from advantagescope.
 */
fun tunable(default: Double, key: String? = null, onChange: (Double) -> Unit = {}): Tunable<Double> {
    return Tunable(key, default, SmartDashboard::getNumber, SmartDashboard::putNumber, onChange)
}

/**
 * A property delegate for a [Boolean] that can be tuned from advantagescope.
 */
fun tunable(default: Boolean, key: String? = null, onChange: (Boolean) -> Unit) =
    Tunable(key, default, SmartDashboard::getBoolean, SmartDashboard::putBoolean, onChange)

/**
 * A property delegate for a [Quantity] that can be tuned from advantagescope.
 */
fun <D: AnyDimension> tunable(default: Quantity<D>, key: String? = null, unit: Quantity<D>, onChange: (Quantity<D>) -> Unit) =
    Tunable(key, default,
        { k, currDefault -> SmartDashboard.getNumber(k, currDefault.inUnit(unit)).ofUnit(unit) },
        { k, v -> SmartDashboard.putNumber(k, v.inUnit(unit)) },
        onChange
    )

class Tunable<T>(
    private var key: String?,
    private var current: T,
    private val get: (String, T) -> T,
    private val put: (String, T) -> Unit,
    private val onChange: (T) -> Unit = {}
) {
    companion object{
        var tuningMode: Boolean = false
    }

    // used to set the property name
    operator fun provideDelegate(thisRef: Any, property: KProperty<*>): Tunable<T> {
        val path = if (key != null) key!! else "${thisRef::class.simpleName}/${property.name}"
        // waits to put the value to the dashboard so that SmartDashboard can initialize
        WaitCommand(0.1)
            .andThen(InstantCommand({ put(path, current) }))
            .ignoringDisable(true)
            .schedule()
        Trigger { tuningMode && get(path, current) != current }
            .whileTrue(InstantCommand({
                current = get(path, current)
                onChange(current)
            }))
        return this
    }
    operator fun getValue(thisRef: Any, property: KProperty<*>): T = current
    operator fun setValue(thisRef: Any, property: KProperty<*>, value: T) { current = value }
}
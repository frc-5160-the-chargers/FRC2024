package frc.chargers.advantagekitextensions

/*
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.util.WPISerializable
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.*

class Test: LoggableInputs {
    fun <T: Any?> logged(default: T, name: String? = null): PropertyDelegateProvider<Any?, ReadWriteProperty<Any?,T>> =
        PropertyDelegateProvider{ _, property -> LoggedInput(default, name ?: property.name) }


    private val allLoggedInputs: MutableList<LoggedInput<*>> = mutableListOf()

    private inner class LoggedInput<T>(private var baseValue: T, val name: String): ReadWriteProperty<Any?, T> {
        init{
            allLoggedInputs.add(this)
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): T = baseValue

        override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
            baseValue = value
        }
    }

    private val dummyValue = Unit
    private val dummyKProperty = ::dummyValue


    override fun toLog(table: LogTable) {
        for (loggedInput in allLoggedInputs){
            val value = loggedInput.getValue(this, dummyKProperty)
            val name = loggedInput.name
            when(value){
                is Int -> table.put(name, value)

                is Double -> table.put(name, value)

                is Quantity<*> -> table.put("$name(SI Value)", value.siValue)

                is WPISerializable -> table.put(name, value)

                is AdvantageKitLoggable<*> -> value.pushToLog(table, name)
            }
        }
    }

    override fun fromLog(table: LogTable) {

    }

}

 */
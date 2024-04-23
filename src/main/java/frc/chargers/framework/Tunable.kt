package frc.chargers.framework

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.Tunable.HasChangedObserver
import frc.chargers.framework.Tunable.RefreshableField
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty


interface Tunable {
    val namespace: String

    /**
     * Represents a tunable value/field whose value can be refreshed.
     */
    fun interface RefreshableField {
        fun refresh()
    }

    /**
     * Represents an observer that detects if a certain group of tunables needs their value refreshed.
     */
    fun interface HasChangedObserver {
        fun hasChanged(): Boolean
    }

    /**
     * Stores a group of tunable fields and "has changed" observers,
     * all united under a single namespace.
     *
     * If any of the observers detect that a value from the dashboard has been changed,
     * it will refresh all the values below.
     */
    class TunableValuesStore {
        private val refreshableFields: MutableList<RefreshableField> = mutableListOf()
        private val hasChangedObservers: MutableList<HasChangedObserver> = mutableListOf()

        fun add(refreshableField: RefreshableField){
            refreshableFields.add(refreshableField)
        }

        fun add(hasChangedObserver: HasChangedObserver){
            hasChangedObservers.add(hasChangedObserver)
        }

        fun refreshWhenNecessary(){
            if (hasChangedObservers.any{ it.hasChanged() }){
                for (field in refreshableFields){
                    field.refresh()
                }
            }
        }
    }



    companion object {
        var tuningMode: Boolean = false
            set(value) {
                if (DriverStation.isFMSAttached() && value){
                    println("Tuning mode was not set; the driver station FMS is attached(you dont want tuning mode in matches lol")
                }else{
                    field = value
                }
            }


        private val tunableStorage: MutableMap<String, TunableValuesStore> = mutableMapOf()

        fun fetchTunableValuesStore(name: String): TunableValuesStore {
            var valuesStore = tunableStorage[name]
            if (valuesStore == null){
                valuesStore = TunableValuesStore()
                tunableStorage[name] = valuesStore
            }
            return valuesStore
        }

        init{
            ChargerRobot.runPeriodically {
                for (tunableValuesStore in tunableStorage.values){
                    tunableValuesStore.refreshWhenNecessary()
                }
            }
        }
    }


    /**
     * A value that simply refreshes itself when a tunableValue is changed,
     * and tuning mode is enabled.
     */
    fun <T: Any?> refreshWhenTuned(getValue: () -> T) =
        object: ReadOnlyProperty<Any?, T>{
            private var value = getValue()

            init{
                fetchTunableValuesStore(namespace).add(
                    RefreshableField {
                        value = getValue()
                    }
                )
            }

            override fun getValue(thisRef: Any?, property: KProperty<*>): T = value
        }

    /**
     * A property delegate that represents a tunable [Double].
     */
    fun tunable(default: Double, key: String? = null) =
        // PropertyDelegateProvider returns a ReadOnlyProperty,
        // which can be delegated to variables to override their getter/setter.
        // We use PropertyDelegateProvider so that the property's name can be provided on startup;
        // allowing the key to be automatically determined.
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Double>> { _, property ->
            object: ReadOnlyProperty<Any?, Double>{
                private var value = default
                private val path = namespace + "/" + (key ?: property.name)

                init{
                    SmartDashboard.putNumber(path, value)
                    fetchTunableValuesStore(namespace).apply{
                        add(RefreshableField { value = SmartDashboard.getNumber(path, value) })
                        add(HasChangedObserver { SmartDashboard.getNumber(path, value) == value })
                    }
                }

                override fun getValue(thisRef: Any?, property: KProperty<*>): Double = value
            }
        }


    /**
     * A property delegate that represents a tunable [Quantity].
     */
    fun <D: Dimension<*, *, *, *>> tunable(default: Quantity<D>, key: String? = null) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Quantity<D>>> { _, property ->
            object: ReadOnlyProperty<Any?, Quantity<D>>{
                private var value = default
                private val path = namespace + "/" + (key ?: property.name) + "(SI Value)"

                init{
                    SmartDashboard.putNumber(path, value.siValue)
                    fetchTunableValuesStore(namespace).apply{
                        add(RefreshableField { value = Quantity(SmartDashboard.getNumber(path, value.siValue)) })
                        add(HasChangedObserver { SmartDashboard.getNumber(path, value.siValue) == value.siValue })
                    }
                }

                override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = value
            }
        }

    /**
     * A property delegate that represents a tunable [Boolean].
     */
    fun tunable(default: Boolean, key: String? = null) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, Boolean>>{ _, property ->
            object: ReadOnlyProperty<Any?, Boolean>{
                private var value = default
                private val path = namespace + "/" + (key ?: property.name)

                init{
                    if (tuningMode) SmartDashboard.putBoolean(path, value)
                    fetchTunableValuesStore(namespace).apply{
                        add(RefreshableField { value = SmartDashboard.getBoolean(path, value) })
                        add(HasChangedObserver { SmartDashboard.getBoolean(path, value) == value })
                    }
                }

                override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = value
            }
        }

    /**
     * Represents [PIDConstants] that can be tuned from the dashboard.
     */
    fun tunable(default: PIDConstants, key: String? = null) =
        PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, PIDConstants>>{ _, property ->
            object: ReadOnlyProperty<Any?, PIDConstants>{
                private var value = default
                private val path = namespace + "/" + (key ?: property.name)

                init{
                    SmartDashboard.putNumber("$path/kP", value.kP)
                    SmartDashboard.putNumber("$path/kI", value.kI)
                    SmartDashboard.putNumber("$path/kD", value.kD)

                    fetchTunableValuesStore(namespace).apply{
                        add(
                            RefreshableField {
                                value = PIDConstants(
                                    SmartDashboard.getNumber("$path/kP", value.kP),
                                    SmartDashboard.getNumber("$path/kI", value.kI),
                                    SmartDashboard.getNumber("$path/kD", value.kD)
                                )
                            }
                        )
                        add(
                            HasChangedObserver {
                                SmartDashboard.getNumber("$path/kP", value.kP) == value.kP &&
                                SmartDashboard.getNumber("$path/kI", value.kI) == value.kI &&
                                SmartDashboard.getNumber("$path/kD", value.kD) == value.kD
                            }
                        )
                    }
                }

                override fun getValue(thisRef: Any?, property: KProperty<*>): PIDConstants = value
            }
        }
}
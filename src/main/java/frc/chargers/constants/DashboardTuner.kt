@file:Suppress("RedundantVisibilityModifier", "unused")
package frc.chargers.constants

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.framework.ChargerRobot
import frc.chargers.utils.math.units.KmeasureUnit
import frc.chargers.utils.math.units.siUnit
import frc.chargers.wpilibextensions.Alert
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitPose3d
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber
import kotlin.properties.PropertyDelegateProvider
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

public typealias TunableDelegate<T> = PropertyDelegateProvider<Any?, ReadOnlyProperty<Any?, T>>


/**
 * A class that interfaces with Smart Dashboard's tuning functionality.
 *
 * Acts as a wrapper around AdvantageKit's [LoggedDashboardNumber] and [LoggedDashboardBoolean],
 * which uses Kotlin's property delegates to support tunable
 * [Double]'s, [Quantity]'s, [Boolean]'s and [PIDConstants].
 *
 * Example usage:
 *
 * ```
 * class BasicSubsystem: SubsystemBase(){
 *      val tuner = DashboardTuner()
 *
 *
 *      val proximalJointConstants by tuner.pidConstants(
 *          default = PIDConstants(0.1,0.0,0.0),
 *          key = "proximalJointPIDConstants"
 *      )
 *
 *      val proximalJointOffset by tuner.quantity(
 *          default = 0.0.degrees,
 *          key = "proximalJointOffsetDegrees",
 *          logUnit = degrees
 *      )
 *
 * }
 * ```
 *
 */
public open class DashboardTuner(
    private val dashKey: String = "TunableValues"
){

    public companion object{

        public var tuningMode: Boolean = false
            set(value){
                if (!value){
                    field = false
                    isCompAlert.active = false
                }else if (DriverStation.isFMSAttached()){
                    isCompAlert.active = true
                }else{
                    field = true
                    tuningModeEnabledAlert.active = true
                    isCompAlert.active = false
                }
            }

        private val isCompAlert =
            Alert.warning(text = "Tuning mode WAS NOT SET: It looks like you're in a match right now.")
        private val tuningModeEnabledAlert =
            Alert.warning(text = "Tuning mode is enabled; Expect loop times to be greater. ")
    }

    init{
        ChargerRobot.runPeriodically {
            if (tuningMode) {
                val updateStatus: List<Boolean> = allTunables.map {
                    it.needsUpdate()
                }

                if (true in updateStatus) {
                    allTunables.forEach {
                        it.updateValue()
                    }
                    allRefreshables.forEach {
                        it.refresh()
                    }
                    println("Values have been refreshed for a Tunable Subsystem.")
                }
            }
        }
    }


    /**
     * A value that simply refreshes itself when a tunableValue is changed,
     * and tuning mode is enabled.
     */
    public fun <T: Any?> refreshWhenTuned(getValue: () -> T): ReadOnlyProperty<Any?, T> =
        object: ReadOnlyProperty<Any?, T> {

            private var value: T = getValue()

            init{
                allRefreshables.add(
                    Refreshable{ value = getValue() }
                )
            }


            override fun getValue(thisRef: Any?, property: KProperty<*>): T = value
        }

    /**
     * A property delegate that represents a tunable [Double].
     *
     * @see LoggedDashboardNumber
     */
    public fun double(default: Double, key: String? = null): TunableDelegate<Double> = PropertyDelegateProvider{
        _, variable -> TunableDouble( default,key ?: variable.name)
    }

    private inner class TunableDouble(default: Double, key: String): ReadOnlyProperty<Any?, Double> {

        val dashNumber = LoggedDashboardNumber("$dashKey/$key", default)
        private var value = default

        init{
            allTunables.add(
                Tunable(
                    {value = dashNumber.get()},
                    {dashNumber.get() == value}
                )
            )
        }


        override fun getValue(thisRef: Any?, property: KProperty<*>): Double = value
    }

    /**
     * A property delegate that represents a tunable [Quantity].
     *
     * @see Quantity
     * @see LoggedDashboardNumber
     */
    public fun <D: Dimension<*, *, *, *>> quantity(
        default: Quantity<D>, key: String? = null, logUnit: KmeasureUnit<D> = siUnit()
    ): TunableDelegate<Quantity<D>> =
        PropertyDelegateProvider{ _, variable ->
            val name = key ?: (variable.name + "(SI unit)")
            TunableQuantity(default, name, logUnit)
        }

    private inner class TunableQuantity<D: Dimension<*, *, *, *>>(
        default: Quantity<D>, key: String, logUnit: KmeasureUnit<D>
    ): ReadOnlyProperty<Any?, Quantity<D>> {

        val dashNumber = LoggedDashboardNumber("$dashKey/$key", default.inUnit(logUnit))
        private var value = default

        init{
            allTunables.add(
                Tunable(
                    {value = dashNumber.get().ofUnit(logUnit)},
                    {dashNumber.get() == value.inUnit(logUnit)}
                )
            )
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> = value

    }

    /**
     * A property delegate that represents a tunable [Boolean].
     *
     * @see LoggedDashboardBoolean
     */
    public fun boolean(default: Boolean, key: String? = null): TunableDelegate<Boolean> =
        PropertyDelegateProvider{ _, variable ->
            TunableBoolean(default, key ?: variable.name)
        }


    private inner class TunableBoolean(
        default: Boolean, key: String
    ): ReadOnlyProperty<Any?, Boolean> {

        val dashBool = LoggedDashboardBoolean(key, default)
        private var value = default

        init{
            allTunables.add(
                Tunable(
                    {value = dashBool.get()},
                    {value == dashBool.get()}
                )
            )
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): Boolean = value
    }

    /**
     * Represents [PIDConstants] that can be tuned from the dashboard.
     */
    public fun pidConstants(default: PIDConstants, key: String? = null): TunableDelegate<PIDConstants> =
        PropertyDelegateProvider{ _, variable ->
            TunablePIDConstants(default, key ?: variable.name)
        }

    /**
     * Represents [PIDConstants] that can be tuned from the dashboard.
     */
    public fun pidConstants(kP: Double, kI: Double, kD: Double, key: String? = null): TunableDelegate<PIDConstants> =
        pidConstants(PIDConstants(kP, kI, kD), key)


    private inner class TunablePIDConstants(
        default: PIDConstants, key: String
    ): ReadOnlyProperty<Any?, PIDConstants> {
        val kpDashNumber = LoggedDashboardNumber("$dashKey/$key-kP", default.kP)
        val kiDashNumber = LoggedDashboardNumber("$dashKey/$key-kI", default.kI)
        val kdDashNumber = LoggedDashboardNumber("$dashKey/$key-kD", default.kD)

        private var value = default
        // ap test stuff; cs and math
        private fun getConstants() = PIDConstants(
            kpDashNumber.get(),
            kiDashNumber.get(),
            kdDashNumber.get()
        )

        init{
            allTunables.add(
                Tunable(
                    { value = getConstants() },
                    { value != getConstants() }
                )
            )
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): PIDConstants = value

    }


    public fun transform3d(
        default: UnitTransform3d,
        key: String? = null
    ): TunableDelegate<UnitTransform3d> =
        PropertyDelegateProvider { _, variable ->
            object: ReadOnlyProperty<Any?, UnitTransform3d>{
                private val tunablePose = TunableUnitPose3d(UnitPose3d() + default, key ?: variable.name)

                override fun getValue(thisRef: Any?, property: KProperty<*>): UnitTransform3d =
                    tunablePose.getValue(thisRef, property) - UnitPose3d()
            }
        }

    public fun pose3d(
        default: UnitPose3d,
        key: String? = null
    ): TunableDelegate<UnitPose3d> =
    PropertyDelegateProvider{ _, variable ->
        TunableUnitPose3d(default, key ?: variable.name)
    }


    private inner class TunableUnitPose3d(
        default: UnitPose3d,
        key: String
    ): ReadOnlyProperty<Any?, UnitPose3d> {
        val xDashNumber = LoggedDashboardNumber("$dashKey/$key-x", default.x.siValue)
        val yDashNumber = LoggedDashboardNumber("$dashKey/$key-y", default.y.siValue)
        val zDashNumber = LoggedDashboardNumber("$dashKey/$key-z", default.z.siValue)

        val yawDashNumber = LoggedDashboardNumber("$dashKey/$key-yaw", default.rotation.x)
        val pitchDashNumber = LoggedDashboardNumber("$dashKey/$key-pitch", default.rotation.y)
        val rollDashNumber = LoggedDashboardNumber("$dashKey/$key-roll", default.rotation.z)

        private var value = default
        // ap test stuff; cs and math
        private fun getPose(): UnitPose3d =
            UnitPose3d(
                Pose3d(
                    Translation3d(xDashNumber.get(), yDashNumber.get(), zDashNumber.get()),
                    Rotation3d(yawDashNumber.get(), pitchDashNumber.get(), rollDashNumber.get())
                )
            )

        init{
            allTunables.add(
                Tunable(
                    { value = getPose() },
                    { value != getPose() }
                )
            )
        }

        override fun getValue(thisRef: Any?, property: KProperty<*>): UnitPose3d = value
    }





    /**
     * represents a generic value that can be refreshed.
     */
    private class Tunable(
        val updateValue: () -> Unit,
        val needsUpdate: () -> Boolean
    )

    /**
     * Represents a generic item which depends on tunables, and can be refreshed when nessecary.
     */
    private fun interface Refreshable{
        fun refresh()
    }

    private val allTunables: MutableList<Tunable> = mutableListOf()
    private val allRefreshables: MutableList<Refreshable> = mutableListOf()

}
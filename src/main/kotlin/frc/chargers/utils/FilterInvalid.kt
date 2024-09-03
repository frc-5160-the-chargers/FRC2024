package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.experimental.ExperimentalTypeInference
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/**
 * Creates a [Quantity] getter that filters NaN and infinity
 */
@OptIn(ExperimentalTypeInference::class)
@OverloadResolutionByLambdaReturnType
fun <D: AnyDimension> filterNaN(getter: () -> Quantity<D>) = object: ReadOnlyProperty<Any?, Quantity<D>> {
    private var previousValue = Quantity<D>(0.0)

    override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> {
        val value = getter()
        if (value.siValue.isNaN() || value.siValue.isInfinite()){
            return previousValue
        }else{
            previousValue = value
            return value
        }
    }
}





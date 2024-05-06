package frc.chargers.utils

import com.batterystaple.kmeasure.dimensions.AnyDimension
import com.batterystaple.kmeasure.quantities.Quantity
import kotlin.experimental.ExperimentalTypeInference
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/**
 * Creates a [Quantity] getter that filters invalid values.
 */
@OptIn(ExperimentalTypeInference::class)
@OverloadResolutionByLambdaReturnType
fun <D: AnyDimension> filterInvalid(
    additionalInvalidFilter: (Quantity<D>) -> Boolean = {false},
    getter: () -> Quantity<D>
) = object: ReadOnlyProperty<Any?, Quantity<D>> {
    private var previousValue = Quantity<D>(0.0)

    override fun getValue(thisRef: Any?, property: KProperty<*>): Quantity<D> {
        val value = getter()
        if (value.siValue.isNaN() || value.siValue.isInfinite() || additionalInvalidFilter(value)){
            return previousValue
        }else{
            previousValue = value
            return value
        }
    }
}





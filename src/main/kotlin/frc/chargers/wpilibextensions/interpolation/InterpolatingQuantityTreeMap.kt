@file:Suppress("unused")
package frc.chargers.wpilibextensions.interpolation

import com.batterystaple.kmeasure.dimensions.Dimension
import com.batterystaple.kmeasure.quantities.Quantity
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator

class InterpolatingQuantityTreeMap<K: Dimension<*,*,*,*>, V: Dimension<*,*,*,*>>:
    InterpolatingTreeMap<Quantity<K>, Quantity<V>>(
        InverseInterpolator{ startValue: Quantity<K>, endValue: Quantity<K>, q: Quantity<K> ->
            MathUtil.inverseInterpolate(startValue.siValue, endValue.siValue, q.siValue)
        },
        Interpolator{ startValue: Quantity<V>, endValue: Quantity<V>, q: Double ->
            Quantity(MathUtil.interpolate(startValue.siValue, endValue.siValue, q))
        }
    )
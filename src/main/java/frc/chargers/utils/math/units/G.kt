@file:Suppress("RedundantVisibilityModifier", "unused") 
package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.quantities.Acceleration
import com.batterystaple.kmeasure.quantities.div
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds

public val g: Acceleration = 9.80665.ofUnit(meters / seconds / seconds)
public val Int.g: Acceleration get() = ofUnit(frc.chargers.utils.math.units.g)
public val Double.g: Acceleration get() = ofUnit(frc.chargers.utils.math.units.g)
public val Long.g: Acceleration get() = ofUnit(frc.chargers.utils.math.units.g)
package frc.chargers.utils.math.units

import com.batterystaple.kmeasure.quantities.MomentOfInertia
import com.batterystaple.kmeasure.quantities.div
import kotlin.math.pow

/**
 * Removes Gear Ratio compensation from a [MomentOfInertia].
 *
 * This allows you to get the moment of inertia of a base motor without gear ratio compensation.
 *
 * More often than not, moment of inertia found online is compensated with the gear ratio.
 */
@Suppress("unused")
fun MomentOfInertia.withoutGearRatio(gearRatio: Double): MomentOfInertia =
    this / gearRatio.pow(2)